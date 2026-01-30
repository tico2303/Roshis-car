#!/usr/bin/env python3
from __future__ import annotations

import json
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class DriveConfig:
    # Convert Twist -> thr/str (int)
    thr_max: int = 100
    str_max: int = 100

    # Scales from Twist units to [-1..1] before mapping to ints
    # For manual teleop, Twist values are typically already scaled, but keep these for tuning.
    linear_x_scale: float = 1.0
    angular_z_scale: float = 1.0

    deadband_thr: int = 3
    deadband_str: int = 3

    # Safety
    cmd_timeout_s: float = 0.25
    publish_hz: float = 20.0

    # Optional sign flips
    thr_sign: int = 1
    str_sign: int = 1


class TwistToDrvNode(Node):
    """
    Subscribes to /cmd_vel and publishes NDJSON drive commands to /esp32/tx_json.

    Output format:
      {"type":"drv","thr":<int>,"str":<int>}
    """

    def __init__(self) -> None:
        super().__init__("twist_to_drv")

        # Params
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tx_topic", "/esp32/tx_json")

        self.declare_parameter("thr_max", 100)
        self.declare_parameter("str_max", 100)
        self.declare_parameter("linear_x_scale", 1.0)
        self.declare_parameter("angular_z_scale", 1.0)
        self.declare_parameter("deadband_thr", 3)
        self.declare_parameter("deadband_str", 3)
        self.declare_parameter("cmd_timeout_s", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("thr_sign", 1)
        self.declare_parameter("str_sign", 1)

        self.cfg = DriveConfig(
            thr_max=int(self.get_parameter("thr_max").value),
            str_max=int(self.get_parameter("str_max").value),
            linear_x_scale=float(self.get_parameter("linear_x_scale").value),
            angular_z_scale=float(self.get_parameter("angular_z_scale").value),
            deadband_thr=int(self.get_parameter("deadband_thr").value),
            deadband_str=int(self.get_parameter("deadband_str").value),
            cmd_timeout_s=float(self.get_parameter("cmd_timeout_s").value),
            publish_hz=float(self.get_parameter("publish_hz").value),
            thr_sign=int(self.get_parameter("thr_sign").value),
            str_sign=int(self.get_parameter("str_sign").value),
        )

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        tx_topic = str(self.get_parameter("tx_topic").value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._sub = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, qos)
        self._pub = self.create_publisher(String, tx_topic, 10)

        self._last_rx_time = 0.0
        self._last_thr = 0
        self._last_str = 0

        period = 1.0 / max(1.0, self.cfg.publish_hz)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"Subscribing: {cmd_vel_topic}")
        self.get_logger().info(f"Publishing:  {tx_topic}")
        self.get_logger().info(
            f"thr_max={self.cfg.thr_max} str_max={self.cfg.str_max} "
            f"timeout={self.cfg.cmd_timeout_s}s hz={self.cfg.publish_hz}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.monotonic()
        self._last_rx_time = now

        # Normalize to [-1..1] then map to integer ranges.
        lx = clamp(msg.linear.x * self.cfg.linear_x_scale, -1.0, 1.0)
        az = clamp(msg.angular.z * self.cfg.angular_z_scale, -1.0, 1.0)

        thr = int(round(lx * self.cfg.thr_max)) * self.cfg.thr_sign
        st  = int(round(az * self.cfg.str_max)) * self.cfg.str_sign

        # Deadband
        if abs(thr) < self.cfg.deadband_thr:
            thr = 0
        if abs(st) < self.cfg.deadband_str:
            st = 0

        self._last_thr = int(clamp(thr, -self.cfg.thr_max, self.cfg.thr_max))
        self._last_str = int(clamp(st, -self.cfg.str_max, self.cfg.str_max))

    def _tick(self) -> None:
        # Safety: if no cmd_vel recently, force stop
        now = time.monotonic()
        timed_out = (now - self._last_rx_time) > self.cfg.cmd_timeout_s
        thr = 0 if timed_out else self._last_thr
        st = 0 if timed_out else self._last_str

        payload = {"type": "drv", "thr": int(thr), "str": int(st)}
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = TwistToDrvNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()