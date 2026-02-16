#!/usr/bin/env python3
"""
Convert geometry_msgs/Twist to differential drive commands (drv2).

Subscribes to /cmd_vel and publishes NDJSON to /esp32/tx_json.

Output format:
  {"type":"drv2","left":<float -1..1>,"right":<float -1..1>}
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def sign_f(x: float) -> float:
    return 1.0 if x > 0 else -1.0 if x < 0 else 0.0


@dataclass
class DriveConfig:
    """Core drive mapping + safety timing."""
    linear_x_scale: float = 1.0
    angular_z_scale: float = 1.0

    deadband: float = 0.03       # minimum magnitude (normalized 0..1)

    cmd_timeout_s: float = 0.05
    publish_hz: float = 50.0

    linear_sign: int = 1
    angular_sign: int = 1


@dataclass
class AxisSlewConfig:
    """
    Slew limits in normalized units per second (range is -1.0 to 1.0).
    """
    slew_up: float = 3.0         # increasing magnitude
    slew_down: float = 6.0       # decreasing magnitude
    slew_reverse: float = 2.0    # sign changes
    reverse_brake_ms: int = 60   # forced zero window during reversal


@dataclass
class AxisState:
    """Per-axis command + smoothed output + reversal brake window."""
    cmd: float = 0.0
    out: float = 0.0
    reverse_until: float = 0.0


class TwistToDrvNode(Node):
    """
    Subscribes to /cmd_vel and publishes NDJSON drv2 commands to /esp32/tx_json.

    Twist conversion (differential drive):
      left  = linear.x - angular.z
      right = linear.x + angular.z
    Both clamped to [-1.0, 1.0].
    """

    def __init__(self) -> None:
        super().__init__("twist_to_drv")

        # -------------------- Params --------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tx_topic", "/esp32/tx_json")

        self.declare_parameter("linear_x_scale", 1.0)
        self.declare_parameter("angular_z_scale", 1.0)
        self.declare_parameter("deadband", 0.03)
        self.declare_parameter("cmd_timeout_s", 0.05)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("linear_sign", 1)
        self.declare_parameter("angular_sign", 1)

        # Left wheel slew
        self.declare_parameter("left_slew_up", 3.0)
        self.declare_parameter("left_slew_down", 6.0)
        self.declare_parameter("left_slew_reverse", 2.0)
        self.declare_parameter("left_reverse_brake_ms", 60)

        # Right wheel slew
        self.declare_parameter("right_slew_up", 3.0)
        self.declare_parameter("right_slew_down", 6.0)
        self.declare_parameter("right_slew_reverse", 2.0)
        self.declare_parameter("right_reverse_brake_ms", 60)

        self.cfg = self._load_drive_config()
        self.left_slew = self._load_axis_slew("left")
        self.right_slew = self._load_axis_slew("right")

        self._period_s = 1.0 / max(1.0, self.cfg.publish_hz)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        tx_topic = str(self.get_parameter("tx_topic").value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._sub = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, qos)
        self._pub = self.create_publisher(String, tx_topic, 10)

        # -------------------- State --------------------
        self._last_rx_time = 0.0
        self._left = AxisState()
        self._right = AxisState()

        self._timer = self.create_timer(self._period_s, self._tick)

        # -------------------- Logs --------------------
        self.get_logger().info(f"Subscribing: {cmd_vel_topic}")
        self.get_logger().info(f"Publishing:  {tx_topic}  (drv2 format)")
        self.get_logger().info(
            f"deadband={self.cfg.deadband} "
            f"timeout={self.cfg.cmd_timeout_s}s hz={self.cfg.publish_hz}"
        )

    # -------------------- Param loading --------------------

    def _load_drive_config(self) -> DriveConfig:
        return DriveConfig(
            linear_x_scale=float(self.get_parameter("linear_x_scale").value),
            angular_z_scale=float(self.get_parameter("angular_z_scale").value),
            deadband=float(self.get_parameter("deadband").value),
            cmd_timeout_s=float(self.get_parameter("cmd_timeout_s").value),
            publish_hz=float(self.get_parameter("publish_hz").value),
            linear_sign=int(self.get_parameter("linear_sign").value),
            angular_sign=int(self.get_parameter("angular_sign").value),
        )

    def _load_axis_slew(self, prefix: str) -> AxisSlewConfig:
        return AxisSlewConfig(
            slew_up=max(0.1, float(self.get_parameter(f"{prefix}_slew_up").value)),
            slew_down=max(0.1, float(self.get_parameter(f"{prefix}_slew_down").value)),
            slew_reverse=max(0.1, float(self.get_parameter(f"{prefix}_slew_reverse").value)),
            reverse_brake_ms=max(0, int(self.get_parameter(f"{prefix}_reverse_brake_ms").value)),
        )

    # -------------------- CmdVel handling --------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_rx_time = time.monotonic()
        left_cmd, right_cmd = self._twist_to_diff(msg)
        self._left.cmd = left_cmd
        self._right.cmd = right_cmd

    def _twist_to_diff(self, msg: Twist) -> tuple[float, float]:
        """Convert Twist to differential drive commands in [-1.0, 1.0]."""
        lx = clamp(msg.linear.x * self.cfg.linear_x_scale, -1.0, 1.0) * self.cfg.linear_sign
        az = clamp(msg.angular.z * self.cfg.angular_z_scale, -1.0, 1.0) * self.cfg.angular_sign

        left = clamp(lx - az, -1.0, 1.0)
        right = clamp(lx + az, -1.0, 1.0)

        db = self.cfg.deadband
        if abs(left) < db:
            left = 0.0
        if abs(right) < db:
            right = 0.0

        return left, right

    # -------------------- Tick --------------------

    def _tick(self) -> None:
        now = time.monotonic()

        if self._is_timed_out(now):
            left_cmd = 0.0
            right_cmd = 0.0
            self._left.reverse_until = 0.0
            self._right.reverse_until = 0.0
        else:
            left_cmd = self._left.cmd
            right_cmd = self._right.cmd

        self._left.out = self._apply_axis_smoothing(now, left_cmd, self._left, self.left_slew)
        self._right.out = self._apply_axis_smoothing(now, right_cmd, self._right, self.right_slew)

        self._publish_drive(self._left.out, self._right.out)

    def _is_timed_out(self, now: float) -> bool:
        return (now - self._last_rx_time) > self.cfg.cmd_timeout_s

    # -------------------- Smoothing --------------------

    def _apply_axis_smoothing(
        self, now: float, cmd: float, state: AxisState, cfg: AxisSlewConfig
    ) -> float:
        state.reverse_until = self._update_reverse_until(
            now=now,
            cmd=cmd,
            out=state.out,
            reverse_until=state.reverse_until,
            brake_ms=cfg.reverse_brake_ms,
        )

        cmd_effective = 0.0 if now < state.reverse_until else cmd

        step = self._max_step(cmd_effective, state.out, cfg)
        return self._step_toward(state.out, cmd_effective, step)

    def _update_reverse_until(
        self, now: float, cmd: float, out: float, reverse_until: float, brake_ms: int
    ) -> float:
        is_reversal = (
            abs(cmd) > 1e-6
            and abs(out) > 1e-6
            and sign_f(cmd) != sign_f(out)
        )
        if not is_reversal:
            return reverse_until

        brake_s = max(0.0, brake_ms / 1000.0)
        return max(reverse_until, now + brake_s)

    def _max_step(self, cmd: float, out: float, cfg: AxisSlewConfig) -> float:
        dt = self._period_s

        is_reversal = (
            abs(cmd) > 1e-6
            and abs(out) > 1e-6
            and sign_f(cmd) != sign_f(out)
        )
        if is_reversal:
            rate = cfg.slew_reverse
        else:
            rate = cfg.slew_up if abs(cmd) > abs(out) else cfg.slew_down

        return max(1e-6, rate * dt)

    def _step_toward(self, current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + (max_step if delta > 0 else -max_step)

    # -------------------- Publish --------------------

    def _publish_drive(self, left: float, right: float) -> None:
        payload = {
            "type": "drv2",
            "left": round(left, 4),
            "right": round(right, 4),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
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
