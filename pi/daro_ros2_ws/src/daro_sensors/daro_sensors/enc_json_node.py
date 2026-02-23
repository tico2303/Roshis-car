#!/usr/bin/env python3
"""
Convert ESP32 encoder NDJSON messages to sensor_msgs/JointState.

Subscribes to /esp32/rx_json/enc and publishes sensor_msgs/JointState
on /wheel/joint_states.  Standard message type consumed by
robot_state_publisher, diff_drive controllers, and SLAM toolboxes.

Expected input JSON (from ESP32 protocol.cpp sendDriveTelemetry()):
{
  "type": "enc",
  "seq": 10,
  "t_ms": 12345,
  "left":  {"dt": 5, "tot": 1200, "rad_s": 2.5},
  "right": {"dt": 4, "tot": 1100, "rad_s": 2.3}
}
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class EncJsonNode(Node):
    def __init__(self):
        super().__init__("enc_json_node")

        # -------- Parameters --------
        self.declare_parameter("rx_topic", "/esp32/rx_json/enc")
        self.declare_parameter("pub_topic", "/wheel/joint_states")
        self.declare_parameter("use_ros_time", True)
        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")
        self.declare_parameter("ticks_per_rev", 660.0)
        # Sign correction for physically reversed motors/encoders (+1 or -1)
        self.declare_parameter("left_sign", 1)
        self.declare_parameter("right_sign", 1)

        rx_topic = str(self.get_parameter("rx_topic").value)
        pub_topic = str(self.get_parameter("pub_topic").value)

        self.create_subscription(String, rx_topic, self._on_enc, 10)
        self._pub = self.create_publisher(JointState, pub_topic, 10)

        self.get_logger().info(
            f"Encoder bridge: {rx_topic} -> {pub_topic}"
        )

    # ------------------------------------------------------------------
    def _on_enc(self, msg: String):
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON: {e}", throttle_duration_sec=5.0)
            return

        try:
            js = self._build_joint_state(obj)
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f"Enc parse error: {e}", throttle_duration_sec=5.0)
            return

        self._pub.publish(js)

    # ------------------------------------------------------------------
    def _build_joint_state(self, obj: dict) -> JointState:
        js = JointState()

        # --- Header ---
        if self.get_parameter("use_ros_time").value:
            js.header.stamp = self.get_clock().now().to_msg()
        else:
            t_ms = int(obj.get("t_ms", 0))
            js.header.stamp.sec = t_ms // 1000
            js.header.stamp.nanosec = (t_ms % 1000) * 1_000_000

        left_name = str(self.get_parameter("left_joint").value)
        right_name = str(self.get_parameter("right_joint").value)
        ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        rads_per_tick = (2.0 * math.pi) / ticks_per_rev
        left_sign = float(self.get_parameter("left_sign").value)
        right_sign = float(self.get_parameter("right_sign").value)

        left = obj["left"]
        right = obj["right"]

        js.name = [left_name, right_name]

        # Position: total ticks -> radians (sign-corrected for motor orientation)
        js.position = [
            left_sign * float(left["tot"]) * rads_per_tick,
            right_sign * float(right["tot"]) * rads_per_tick,
        ]

        # Velocity: rad/s (already computed on ESP32, sign-corrected)
        js.velocity = [
            left_sign * float(left["rad_s"]),
            right_sign * float(right["rad_s"]),
        ]

        # Effort: not measured
        js.effort = []

        return js


def main():
    rclpy.init()
    node = None
    try:
        node = EncJsonNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
