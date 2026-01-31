#!/usr/bin/env python3
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


def sign_i(x: int) -> int:
    return 1 if x > 0 else -1 if x < 0 else 0


@dataclass
class DriveConfig:
    """Core drive mapping + safety timing."""
    thr_max: int = 100
    str_max: int = 100

    linear_x_scale: float = 1.0
    angular_z_scale: float = 1.0

    deadband_thr: int = 3
    deadband_str: int = 3

    cmd_timeout_s: float = 0.05
    publish_hz: float = 50.0

    thr_sign: int = 1
    str_sign: int = 1


@dataclass
class AxisSlewConfig:
    """
    Slew limits are in "axis units per second" where the axis unit is the integer
    you send to the ESP32 (e.g., thr in [-thr_max..thr_max]).
    """
    slew_up: float = 200.0        # increasing magnitude (0 -> 80)
    slew_down: float = 400.0      # decreasing magnitude (80 -> 0)
    slew_reverse: float = 120.0   # sign changes (+ -> -)
    reverse_brake_ms: int = 60    # forced 0 window during reversal


@dataclass
class AxisState:
    """Per-axis command + smoothed output + reversal brake window."""
    cmd: int = 0
    out: int = 0
    reverse_until: float = 0.0


class TwistToDrvNode(Node):
    """
    Subscribes to /cmd_vel and publishes NDJSON drive commands to /esp32/tx_json.

    Output format:
      {"type":"drv","thr":<int>,"str":<int>}
    """

    def __init__(self) -> None:
        super().__init__("twist_to_drv")

        # -------------------- Params --------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tx_topic", "/esp32/tx_json")

        self.declare_parameter("thr_max", 100)
        self.declare_parameter("str_max", 100)
        self.declare_parameter("linear_x_scale", 1.0)
        self.declare_parameter("angular_z_scale", 1.0)
        self.declare_parameter("deadband_thr", 3)
        self.declare_parameter("deadband_str", 3)
        self.declare_parameter("cmd_timeout_s", 0.05)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("thr_sign", 1)
        self.declare_parameter("str_sign", 1)

        # Throttle slew
        self.declare_parameter("thr_slew_up", 200.0)
        self.declare_parameter("thr_slew_down", 400.0)
        self.declare_parameter("thr_slew_reverse", 120.0)
        self.declare_parameter("thr_reverse_brake_ms", 60)

        # Steering slew (for differential drive, steering changes can spike current too)
        self.declare_parameter("str_slew_up", 250.0)
        self.declare_parameter("str_slew_down", 500.0)
        self.declare_parameter("str_slew_reverse", 150.0)
        self.declare_parameter("str_reverse_brake_ms", 40)

        self.cfg = self._load_drive_config()
        self.thr_slew = self._load_axis_slew("thr", AxisSlewConfig(200.0, 400.0, 120.0, 60))
        self.str_slew = self._load_axis_slew("str", AxisSlewConfig(250.0, 500.0, 150.0, 40))

        self._period_s = 1.0 / max(1.0, self.cfg.publish_hz)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        tx_topic = str(self.get_parameter("tx_topic").value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._sub = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, qos)
        self._pub = self.create_publisher(String, tx_topic, 10)

        # -------------------- State --------------------
        self._last_rx_time = 0.0
        self._thr = AxisState()
        self._str = AxisState()

        self._timer = self.create_timer(self._period_s, self._tick)

        # -------------------- Logs --------------------
        self.get_logger().info(f"Subscribing: {cmd_vel_topic}")
        self.get_logger().info(f"Publishing:  {tx_topic}")
        self.get_logger().info(
            f"thr_max={self.cfg.thr_max} str_max={self.cfg.str_max} "
            f"timeout={self.cfg.cmd_timeout_s}s hz={self.cfg.publish_hz}"
        )
        self.get_logger().info(
            f"thr_slew(up/down/rev/brake_ms)="
            f"{self.thr_slew.slew_up}/{self.thr_slew.slew_down}/{self.thr_slew.slew_reverse}/{self.thr_slew.reverse_brake_ms}"
        )
        self.get_logger().info(
            f"str_slew(up/down/rev/brake_ms)="
            f"{self.str_slew.slew_up}/{self.str_slew.slew_down}/{self.str_slew.slew_reverse}/{self.str_slew.reverse_brake_ms}"
        )

    # -------------------- Param loading --------------------

    def _load_drive_config(self) -> DriveConfig:
        return DriveConfig(
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

    def _load_axis_slew(self, prefix: str, default: AxisSlewConfig) -> AxisSlewConfig:
        """
        Load axis slew config from parameters:
          <prefix>_slew_up, <prefix>_slew_down, <prefix>_slew_reverse, <prefix>_reverse_brake_ms
        """
        up = float(self.get_parameter(f"{prefix}_slew_up").value)
        down = float(self.get_parameter(f"{prefix}_slew_down").value)
        rev = float(self.get_parameter(f"{prefix}_slew_reverse").value)
        brake = int(self.get_parameter(f"{prefix}_reverse_brake_ms").value)

        # Sanity clamps
        up = up if up > 0 else default.slew_up
        down = down if down > 0 else default.slew_down
        rev = rev if rev > 0 else default.slew_reverse
        brake = brake if brake >= 0 else default.reverse_brake_ms

        return AxisSlewConfig(up, down, rev, brake)

    # -------------------- CmdVel handling --------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_rx_time = time.monotonic()
        thr_cmd, str_cmd = self._twist_to_thr_str(msg)
        self._thr.cmd = thr_cmd
        self._str.cmd = str_cmd

    def _twist_to_thr_str(self, msg: Twist) -> tuple[int, int]:
        """Convert Twist -> (thr_cmd, str_cmd) ints with scaling + deadbands applied."""
        lx = clamp(msg.linear.x * self.cfg.linear_x_scale, -1.0, 1.0)
        az = clamp(msg.angular.z * self.cfg.angular_z_scale, -1.0, 1.0)

        thr = int(round(lx * self.cfg.thr_max)) * self.cfg.thr_sign
        st = int(round(az * self.cfg.str_max)) * self.cfg.str_sign

        if abs(thr) < self.cfg.deadband_thr:
            thr = 0
        if abs(st) < self.cfg.deadband_str:
            st = 0

        thr = int(clamp(thr, -self.cfg.thr_max, self.cfg.thr_max))
        st = int(clamp(st, -self.cfg.str_max, self.cfg.str_max))
        return thr, st

    # -------------------- Tick --------------------

    def _tick(self) -> None:
        now = time.monotonic()

        if self._is_timed_out(now):
            thr_cmd = 0
            str_cmd = 0
            # Clear brake windows so we don't "stick" after timeout
            self._thr.reverse_until = 0.0
            self._str.reverse_until = 0.0
        else:
            thr_cmd = self._thr.cmd
            str_cmd = self._str.cmd

        self._thr.out = self._apply_axis_smoothing(now, thr_cmd, self._thr, self.thr_slew)
        self._str.out = self._apply_axis_smoothing(now, str_cmd, self._str, self.str_slew)

        self._publish_drive(self._thr.out, self._str.out)

    def _is_timed_out(self, now: float) -> bool:
        return (now - self._last_rx_time) > self.cfg.cmd_timeout_s

    # -------------------- Smoothing --------------------

    def _apply_axis_smoothing(self, now: float, cmd: int, state: AxisState, cfg: AxisSlewConfig) -> int:
        """
        Smooth an axis command:
          1) If sign change requested, extend a brake window (force cmd=0 for a short time)
          2) During brake window, cmd is forced to 0
          3) Slew-limit state.out toward cmd
        """
        state.reverse_until = self._update_reverse_until(
            now=now,
            cmd=cmd,
            out=state.out,
            reverse_until=state.reverse_until,
            brake_ms=cfg.reverse_brake_ms,
        )

        cmd_effective = 0 if now < state.reverse_until else cmd

        step = self._max_step(cmd_effective, state.out, cfg)
        return self._step_toward(state.out, cmd_effective, step)

    def _update_reverse_until(self, now: float, cmd: int, out: int, reverse_until: float, brake_ms: int) -> float:
        """
        If a sign change is requested (out and cmd non-zero and opposite signs),
        extend a 'brake-to-zero' window. During that window we force cmd=0.
        """
        is_reversal = cmd != 0 and out != 0 and sign_i(cmd) != sign_i(out)
        if not is_reversal:
            return reverse_until

        brake_s = max(0.0, brake_ms / 1000.0)
        return max(reverse_until, now + brake_s)

    def _max_step(self, cmd: int, out: int, cfg: AxisSlewConfig) -> int:
        """
        Compute max integer change allowed per tick.
        """
        dt = self._period_s

        is_reversal = cmd != 0 and out != 0 and sign_i(cmd) != sign_i(out)
        if is_reversal:
            rate = cfg.slew_reverse
        else:
            # increasing magnitude vs decreasing magnitude
            rate = cfg.slew_up if abs(cmd) > abs(out) else cfg.slew_down

        max_step = int(math.ceil(max(0.0, rate) * dt))
        return max(1, max_step)

    def _step_toward(self, current: int, target: int, max_step: int) -> int:
        if current == target:
            return current

        delta = target - current
        if abs(delta) <= max_step:
            return target

        return current + (max_step if delta > 0 else -max_step)

    # -------------------- Publish --------------------

    def _publish_drive(self, thr: int, st: int) -> None:
        payload = {"type": "drv", "thr": int(thr), "str": int(st)}
        msg = String()
        # compact JSON; ESP32 parsing likes fewer spaces
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