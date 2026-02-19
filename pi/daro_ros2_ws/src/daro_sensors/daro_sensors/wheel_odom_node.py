#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


# -----------------------------
# Small math helpers
# -----------------------------
def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    """
    Convert a planar yaw angle (rotation about Z) into a quaternion.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def stamp_to_seconds(stamp) -> float:
    """
    Convert builtin_interfaces/msg/Time to seconds as float.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class WheelReadings:
    """
    Wheel readings derived from JointState for one side.
    """
    position_rad: float
    velocity_rad_s: float


@dataclass
class WheelDeltas:
    """
    Wheel motion increment between samples (linearized).
    """
    left_m: float
    right_m: float


@dataclass
class BodyTwist:
    """
    Robot body-frame velocities for a differential drive robot.
    """
    linear_x_m_s: float
    angular_z_rad_s: float


# -----------------------------
# Main node
# -----------------------------
class WheelOdomNode(Node):
    """
    Convert wheel JointState into nav_msgs/Odometry using diff-drive kinematics.

    This is "wheel odometry" (a dead-reckoning estimate) and will drift over time.
    It is a great input for robot_localization EKF, and a decent stop-gap for SLAM.
    """

    def __init__(self) -> None:
        super().__init__("wheel_odom_node")

        self._declare_parameters()
        self._load_parameters()

        self._x_m: float = 0.0
        self._y_m: float = 0.0
        self._yaw_rad: float = 0.0

        self._prev_stamp_sec: Optional[float] = None
        self._prev_left_pos_rad: Optional[float] = None
        self._prev_right_pos_rad: Optional[float] = None

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._js_sub = self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 50)

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.get_logger().info(
            "Wheel odom started:\n"
            f"  JointState: {self._joint_states_topic}\n"
            f"  Odometry:   {self._odom_topic}\n"
            f"  Left joint:  {self._left_joint}\n"
            f"  Right joint: {self._right_joint}\n"
            f"  wheel_radius_m={self._wheel_radius_m}, wheel_separation_m={self._wheel_separation_m}\n"
            f"  publish_tf={self._publish_tf} (usually False when EKF publishes TF)"
        )

    # -----------------------------
    # Parameter setup
    # -----------------------------
    def _declare_parameters(self) -> None:
        # Topics
        self.declare_parameter("joint_states_topic", "/wheel/joint_states")
        self.declare_parameter("odom_topic", "/wheel/odom")

        # Joint names inside JointState
        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")

        # Robot geometry (must match your physical robot)
        self.declare_parameter("wheel_radius_m", 0.03)       # radius, not diameter
        self.declare_parameter("wheel_separation_m", 0.16)   # distance between wheel contact points

        # Frames used in the Odometry message (and TF if enabled)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # TF publishing (usually OFF if EKF publishes odom->base_link)
        self.declare_parameter("publish_tf", False)

        # Simple covariances for wheel odom (diagonal entries only)
        self.declare_parameter("pose_cov_xy", 0.02)     # m^2
        self.declare_parameter("pose_cov_yaw", 0.05)    # rad^2
        self.declare_parameter("twist_cov_vx", 0.05)    # (m/s)^2
        self.declare_parameter("twist_cov_wz", 0.10)    # (rad/s)^2

    def _load_parameters(self) -> None:
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._left_joint = str(self.get_parameter("left_joint").value)
        self._right_joint = str(self.get_parameter("right_joint").value)

        self._wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self._wheel_separation_m = float(self.get_parameter("wheel_separation_m").value)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._pose_cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self._pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)
        self._twist_cov_vx = float(self.get_parameter("twist_cov_vx").value)
        self._twist_cov_wz = float(self.get_parameter("twist_cov_wz").value)

        if self._wheel_separation_m <= 0.0:
            raise ValueError("wheel_separation_m must be > 0")
        if self._wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be > 0")

    # -----------------------------
    # ROS callback
    # -----------------------------
    def _on_joint_state(self, msg: JointState) -> None:
        """
        Main pipeline:
          1) Extract left/right wheel readings from JointState
          2) Compute dt and wheel distance deltas
          3) Integrate pose (x,y,yaw)
          4) Publish Odometry (and TF optionally)
        """
        wheel_readings = self._extract_wheel_readings(msg)
        if wheel_readings is None:
            return

        stamp_sec = self._get_message_time_sec(msg)
        if stamp_sec is None:
            return

        dt = self._compute_dt_sec(stamp_sec)
        if dt is None:
            return

        wheel_deltas = self._compute_wheel_deltas_m(wheel_readings)
        if wheel_deltas is None:
            return

        self._integrate_pose(wheel_deltas)

        twist = self._compute_body_twist(wheel_readings)

        odom_msg = self._build_odometry_message(stamp_sec, twist)
        self._odom_pub.publish(odom_msg)

        self._publish_tf_if_enabled(stamp_sec)

    # -----------------------------
    # Extraction / timing
    # -----------------------------
    def _extract_wheel_readings(self, msg: JointState) -> Optional[Tuple[WheelReadings, WheelReadings]]:
        """
        Pull left/right wheel (position, velocity) from JointState.

        Returns None if expected joint names are not present.
        """
        try:
            left_idx = msg.name.index(self._left_joint)
            right_idx = msg.name.index(self._right_joint)
        except ValueError:
            # JointState doesn't include our joints (or names mismatch)
            return None

        left_pos = self._safe_list_get(msg.position, left_idx, default=0.0)
        right_pos = self._safe_list_get(msg.position, right_idx, default=0.0)

        left_vel = self._safe_list_get(msg.velocity, left_idx, default=0.0)
        right_vel = self._safe_list_get(msg.velocity, right_idx, default=0.0)

        return WheelReadings(left_pos, left_vel), WheelReadings(right_pos, right_vel)

    def _get_message_time_sec(self, msg: JointState) -> Optional[float]:
        """
        Prefer the timestamp in JointState. If it's zero, fallback to local clock time.

        Using message time is nicer because it reflects the encoder sampling time, but
        for many hobby setups it may be unset.
        """
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            now = self.get_clock().now().to_msg()
            return stamp_to_seconds(now)
        return stamp_to_seconds(msg.header.stamp)

    def _compute_dt_sec(self, stamp_sec: float) -> Optional[float]:
        """
        Compute dt since last message. Returns None until we have a previous sample.
        """
        if self._prev_stamp_sec is None:
            self._prev_stamp_sec = stamp_sec
            return None

        dt = stamp_sec - self._prev_stamp_sec
        if dt <= 0.0:
            return None

        self._prev_stamp_sec = stamp_sec
        return dt

    # -----------------------------
    # Kinematics
    # -----------------------------
    def _compute_wheel_deltas_m(self, wheels: Tuple[WheelReadings, WheelReadings]) -> Optional[WheelDeltas]:
        """
        Convert wheel angle deltas (radians) into linear travel (meters).
        """
        left, right = wheels

        # First sample: initialize previous wheel positions and skip integration.
        if self._prev_left_pos_rad is None or self._prev_right_pos_rad is None:
            self._prev_left_pos_rad = left.position_rad
            self._prev_right_pos_rad = right.position_rad
            return None

        d_left_rad = left.position_rad - self._prev_left_pos_rad
        d_right_rad = right.position_rad - self._prev_right_pos_rad

        self._prev_left_pos_rad = left.position_rad
        self._prev_right_pos_rad = right.position_rad

        # Convert wheel rotation into distance along the ground
        d_left_m = d_left_rad * self._wheel_radius_m
        d_right_m = d_right_rad * self._wheel_radius_m

        return WheelDeltas(left_m=d_left_m, right_m=d_right_m)

    def _integrate_pose(self, delta: WheelDeltas) -> None:
        """
        Differential drive pose integration.

        ds = average wheel travel
        d_yaw = (right-left)/wheel_separation

        We integrate using "mid-yaw" (a small improvement over naive Euler),
        which reduces drift when turning.
        """
        ds = 0.5 * (delta.left_m + delta.right_m)
        d_yaw = (delta.right_m - delta.left_m) / self._wheel_separation_m

        mid_yaw = self._yaw_rad + 0.5 * d_yaw

        self._x_m += ds * math.cos(mid_yaw)
        self._y_m += ds * math.sin(mid_yaw)
        self._yaw_rad += d_yaw

    def _compute_body_twist(self, wheels: Tuple[WheelReadings, WheelReadings]) -> BodyTwist:
        """
        Convert wheel angular velocities into robot body twist (v, wz).
        """
        left, right = wheels

        v_left = left.velocity_rad_s * self._wheel_radius_m
        v_right = right.velocity_rad_s * self._wheel_radius_m

        v = 0.5 * (v_left + v_right)
        wz = (v_right - v_left) / self._wheel_separation_m

        return BodyTwist(linear_x_m_s=v, angular_z_rad_s=wz)

    # -----------------------------
    # Message building
    # -----------------------------
    def _build_odometry_message(self, stamp_sec: float, twist: BodyTwist) -> Odometry:
        """
        Construct nav_msgs/Odometry. This is the message robot_localization will consume.
        """
        odom = Odometry()

        # Build a ROS Time from stamp_sec
        sec = int(stamp_sec)
        nanosec = int((stamp_sec - sec) * 1e9)

        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec

        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = self._x_m
        odom.pose.pose.position.y = self._y_m
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self._yaw_rad)

        odom.twist.twist.linear.x = twist.linear_x_m_s
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = twist.angular_z_rad_s

        # Simple diagonal covariance. Wheel odom is decent short-term but drifts.
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self._pose_cov_xy   # x
        odom.pose.covariance[7] = self._pose_cov_xy   # y
        odom.pose.covariance[35] = self._pose_cov_yaw # yaw

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self._twist_cov_vx   # vx
        odom.twist.covariance[35] = self._twist_cov_wz  # wz

        return odom

    def _publish_tf_if_enabled(self, stamp_sec: float) -> None:
        """
        If publish_tf is enabled, broadcast odom->base_link.
        WARNING: If EKF publishes odom->base_link, keep publish_tf=False to avoid TF fights.
        """
        if self._tf_broadcaster is None:
            return

        sec = int(stamp_sec)
        nanosec = int((stamp_sec - sec) * 1e9)

        t = TransformStamped()
        t.header.stamp.sec = sec
        t.header.stamp.nanosec = nanosec
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame

        t.transform.translation.x = self._x_m
        t.transform.translation.y = self._y_m
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self._yaw_rad)

        self._tf_broadcaster.sendTransform(t)

    # -----------------------------
    # Utility
    # -----------------------------
    @staticmethod
    def _safe_list_get(values, index: int, default: float) -> float:
        try:
            return float(values[index])
        except Exception:
            return float(default)


def main() -> None:
    rclpy.init()
    node: Optional[WheelOdomNode] = None
    try:
        node = WheelOdomNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()