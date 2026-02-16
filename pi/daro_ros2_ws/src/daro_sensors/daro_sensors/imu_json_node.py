#!/usr/bin/env python3
"""
Convert ESP32 IMU NDJSON messages to sensor_msgs/Imu.

Subscribes to the bridge's /esp32/rx_json/sens topic, filters for
data.sensor == "imu", and publishes sensor_msgs/Imu on /imu/data_raw.

Expected input JSON (from ESP32 bmi160_imu.h publish()):
{
  "type": "sens",
  "seq": 42,
  "t_ms": 12345,
  "data": {
    "sensor": "imu",
    "linear_acceleration": {"x": 0.1, "y": -0.05, "z": 9.81},
    "angular_velocity": {"x": 0.01, "y": -0.02, "z": 0.0}
  }
}
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu


class ImuJsonNode(Node):
    def __init__(self):
        super().__init__("imu_json_node")

        # -------- Parameters --------
        self.declare_parameter("rx_topic", "/esp32/rx_json/sens")
        self.declare_parameter("pub_topic", "/imu/data_raw")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("use_ros_time", True)

        rx_topic = str(self.get_parameter("rx_topic").value)
        pub_topic = str(self.get_parameter("pub_topic").value)

        self.create_subscription(String, rx_topic, self._on_sens, 10)
        self._pub = self.create_publisher(Imu, pub_topic, 10)

        self.get_logger().info(
            f"IMU bridge: {rx_topic} -> {pub_topic}"
        )

    # ------------------------------------------------------------------
    def _on_sens(self, msg: String):
        """Filter for sensor == 'imu' and convert to Imu message."""
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON: {e}", throttle_duration_sec=5.0)
            return

        data = obj.get("data", {})
        if data.get("sensor") != "imu":
            return  # not an IMU message, ignore

        try:
            imu_msg = self._build_imu_msg(obj, data)
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f"IMU parse error: {e}", throttle_duration_sec=5.0)
            return

        self._pub.publish(imu_msg)

    # ------------------------------------------------------------------
    def _build_imu_msg(self, obj: dict, data: dict) -> Imu:
        imu = Imu()

        # --- Header ---
        imu.header.frame_id = str(self.get_parameter("frame_id").value)

        if self.get_parameter("use_ros_time").value:
            imu.header.stamp = self.get_clock().now().to_msg()
        else:
            t_ms = int(obj.get("t_ms", 0))
            imu.header.stamp.sec = t_ms // 1000
            imu.header.stamp.nanosec = (t_ms % 1000) * 1_000_000

        # --- Orientation (not estimated, mark unknown) ---
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 0.0  # all zeros = "no orientation estimate"
        # Covariance[0] = -1 means orientation data should not be used
        imu.orientation_covariance[0] = -1.0

        # --- Angular velocity (rad/s) ---
        av = data["angular_velocity"]
        imu.angular_velocity.x = float(av["x"])
        imu.angular_velocity.y = float(av["y"])
        imu.angular_velocity.z = float(av["z"])
        # Diagonal covariance (reasonable default for BMI160 at ±500dps)
        gyro_var = (math.radians(0.05)) ** 2  # ~0.05 deg/s noise density
        imu.angular_velocity_covariance[0] = gyro_var
        imu.angular_velocity_covariance[4] = gyro_var
        imu.angular_velocity_covariance[8] = gyro_var

        # --- Linear acceleration (m/s²) ---
        la = data["linear_acceleration"]
        imu.linear_acceleration.x = float(la["x"])
        imu.linear_acceleration.y = float(la["y"])
        imu.linear_acceleration.z = float(la["z"])
        # Diagonal covariance (reasonable default for BMI160 at ±4g)
        accel_var = 0.003 ** 2  # ~3 mg noise density
        imu.linear_acceleration_covariance[0] = accel_var
        imu.linear_acceleration_covariance[4] = accel_var
        imu.linear_acceleration_covariance[8] = accel_var

        return imu


def main():
    rclpy.init()
    node = None
    try:
        node = ImuJsonNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
