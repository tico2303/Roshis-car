"""
xbox_drive_bridge.py

Bridge Xbox controller input -> ESP32 drive protocol (NDJSON over serial).

Design goals:
- Clear separation of concerns:
  - Input (controller)
  - Mapping (controller state -> drive commands)
  - Transport (serial protocol)
  - Orchestration (bridge loop)
- Extensible:
  - Swap controller backend (pygame, evdev, etc.) via adapter interface
  - Swap mapping strategy (tank drive, arcade drive, etc.)
  - Add new protocol messages later (mode, ping, etc.)
- Conservative, robust behavior:
  - Deadzone + shaping
  - Rate limiting and change detection
  - Optional "failsafe" stop on disconnect / inactivity

Notes:
- Pygame axis conventions often have LY up as -1.0; we invert to make forward positive.
- This file assumes your `utils.py` provides:
    - now_ms()
    - clamp()
    - apply_deadzone()
    - scale_to_int100()
    - _escape_bytes()
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Protocol as TypingProtocol, Tuple
import json

# Your existing imports/classes:
# - XboxController / XboxControllerState
# - NdjsonSerialProtocol
# - utils: now_ms, clamp, apply_deadzone, scale_to_int100
from app.utils import now_ms, clamp, apply_deadzone, scale_to_int100


# ----------------------------
# Controller abstraction
# ----------------------------
@dataclass(frozen=True)
class ControllerState:
    """
    Normalized controller state.

    Axes are floats in [-1.0, 1.0] where:
      - forward is +1.0 (we invert as needed in adapters)
      - right is +1.0
      - triggers are in [-1,1] or [0,1] depending on backend; adapters should normalize
    """
    forward: float  # forward/back on left stick (or trigger mix). forward = +1
    turn: float     # left/right on left stick. right = +1
    lt: float       # 0..1
    rt: float       # 0..1
    a: bool
    b: bool
    x: bool
    y: bool
    lb: bool
    rb: bool
    menu: bool
    view: bool


class IController(TypingProtocol):
    """
    Controller interface. You can provide alternative implementations (evdev, etc.)
    as long as they implement read_state().
    """
    def read_state(self) -> ControllerState: ...


# ----------------------------
# Mapping: Controller -> DriveCmd
# ----------------------------
@dataclass(frozen=True)
class DriveCmd:
    """
    Drive command in your protocol units.

    thr: -100..100  (forward/back)
    str: -100..100  (left/right)
    ms: duration in milliseconds that ESP32 should apply this command (optional)
    src: string source label
    """
    thr: int
    steer: int
    ms: int = 200
    src: str = "xbox"


@dataclass(frozen=True)
class DriveTuning:
    """
    Tuning parameters for mapping and smoothing.

    deadzone:
      Applied to forward/turn axes (not triggers).
    expo:
      0.0 = linear, 0.5..0.8 = gentler near center, still reaches full scale.
    max_thr / max_steer:
      Caps output for safety.
    min_send_interval_s:
      Rate limit: don't spam serial faster than this.
    min_change:
      Don't send if change is smaller than this threshold (in -100..100 units).
    """
    deadzone: float = 0.12
    expo: float = 0.35
    max_thr: int = 100
    max_steer: int = 100
    min_send_interval_s: float = 0.05  # 20 Hz
    min_change: int = 2                # ignore tiny jitter
    max_stale_s: float = 0.15  # resend at least ~6-7 Hz even if unchanged
    # tuning for current draw from exceleration
    thr_slew_per_s: float = 120.0    # units/sec in -100..100 space
    steer_slew_per_s: float = 200.0


def _expo(x: float, expo: float) -> float:
    """
    Simple expo curve:
      y = (1-expo)*x + expo*x^3
    Keeps sign, preserves endpoints.
    """
    x = clamp(x, -1.0, 1.0)
    return (1.0 - expo) * x + expo * (x * x * x)


class ArcadeDriveMapper:
    """
    Arcade drive: one stick controls forward/back and turn.

    forward axis -> throttle
    turn axis -> steering
    """
    def __init__(self, tuning: DriveTuning) -> None:
        self.tuning = tuning

    def map(self, st: ControllerState) -> DriveCmd:
        f = apply_deadzone(st.forward, self.tuning.deadzone)
        t = apply_deadzone(st.turn, self.tuning.deadzone)

        f = _expo(f, self.tuning.expo)
        t = _expo(t, self.tuning.expo)

        thr = int(round(scale_to_int100(f)))
        steer = int(round(scale_to_int100(t)))

        thr = int(clamp(thr, -self.tuning.max_thr, self.tuning.max_thr))
        steer = int(clamp(steer, -self.tuning.max_steer, self.tuning.max_steer))

        return DriveCmd(thr=thr, steer=steer)


# ----------------------------
# Bridge: orchestrates controller + protocol
# ----------------------------
@dataclass
class BridgeConfig:
    """
    Runtime behavior config.
    """
    hz: float = 50.0
    stop_on_disconnect: bool = True
    stop_on_inactive_s: Optional[float] = 2.0  # send stop if no input changes for N seconds
    heartbeat_s: Optional[float] = None        # e.g. 2.0 to send periodic ping (optional)
    drv_ms: int = 200                          # included in drv messages


class DriveBridge:
    """
    Polls controller state, maps it to drive commands, sends them over the protocol.

    Extensibility points:
    - controller: any IController
    - mapper: any object with map(ControllerState)->DriveCmd
    - protocol: your NdjsonSerialProtocol (or compatible)

    Practical behavior:
    - Sends only on meaningful change (reduces serial spam)
    - Rate-limited
    - Optional inactivity failsafe stop
    """
    def __init__(
        self,
        controller: IController,
        mapper: ArcadeDriveMapper,
        proto: "NdjsonSerialProtocol",
        *,
        cfg: BridgeConfig = BridgeConfig(),
        tuning: DriveTuning = DriveTuning(),
    ) -> None:
        self.controller = controller
        self.mapper = mapper
        self.proto = proto
        self.cfg = cfg
        self.tuning = tuning

        self._last_send_t = 0.0
        self._last_cmd: Optional[DriveCmd] = None
        self._last_activity_t = time.time()
        self._last_heartbeat_t = time.time()

        self._thr_cur = 0.0
        self._steer_cur = 0.0
        self._last_ramp_t = time.time()


    def _should_send(self, cmd: DriveCmd) -> bool:
      now = time.time()
      # rate limit
      if now - self._last_send_t < self.tuning.min_send_interval_s:
        return False

      # always send first command
      if self._last_cmd is None:
        return True

      # resend if stale even if unchanged (keepalive)
      if (now - self._last_send_t) >= self.tuning.max_stale_s:
        return True

     # send if changed enough
      if abs(cmd.thr - self._last_cmd.thr) >= self.tuning.min_change:
        return True
      if abs(cmd.steer - self._last_cmd.steer) >= self.tuning.min_change:
        return True

      return False

    def _send_cmd(self, cmd: DriveCmd) -> None:
        seq = self.proto.send_drv(cmd.thr, cmd.steer, ms=self.cfg.drv_ms, src=cmd.src)
        # Optionally wait for ack; in realtime driving you may *not* want to block.
        # If you want stronger guarantees, enable a non-blocking ack tracker later.
        self._last_cmd = cmd
        self._last_send_t = time.time()
        return

    def _send_stop(self) -> None:
        self._send_cmd(DriveCmd(thr=0, steer=0, src="failsafe"))

    def run_forever(self) -> None:
        """
        Main loop. Ctrl+C to exit.

        Recommended call order:
          - proto.drain(1.0)
          - proto.send_hello(...)
          - bridge.run_forever()
        """
        period = 1.0 / max(1.0, self.cfg.hz)

        while True:
            loop_start = time.time()

            # Optional periodic ping heartbeat (keeps link "chatty")
            if self.cfg.heartbeat_s is not None:
                if (time.time() - self._last_heartbeat_t) >= self.cfg.heartbeat_s:
                    self.proto.send_ping(now_ms())
                    self._last_heartbeat_t = time.time()

            # Read controller
            try:
                st = self.controller.read_state()
            except Exception:
                # If controller read fails, optionally stop robot
                if self.cfg.stop_on_disconnect:
                    self._send_stop()
                raise

            # Map to drive command
            cmd = self.mapper.map(st)

            now = time.time()
            # Please comment (prevents stall from current spike during accerlation)
            dt = max(0.0, now - self._last_ramp_t)
            self._last_ramp_t = now

            def slew(cur: float, target: float, rate: float) -> float:
                max_step = rate * dt
                if target > cur:
                    return min(target, cur + max_step)
                return max(target, cur - max_step)

            self._thr_cur = slew(self._thr_cur, float(cmd.thr), self.tuning.thr_slew_per_s)
            self._steer_cur = slew(self._steer_cur, float(cmd.steer), self.tuning.steer_slew_per_s)
            cmd = DriveCmd(thr=int(round(self._thr_cur)), steer=int(round(self._steer_cur)), ms=cmd.ms, src=cmd.src)
            # end Please comment

            # Activity tracking (for optional failsafe)
            if self._last_cmd is None or (cmd.thr != self._last_cmd.thr or cmd.steer != self._last_cmd.steer):
                self._last_activity_t = time.time()

            # Inactivity failsafe
            if self.cfg.stop_on_inactive_s is not None:
                if (time.time() - self._last_activity_t) >= self.cfg.stop_on_inactive_s:
                    if self._last_cmd is None or self._last_cmd.thr != 0 or self._last_cmd.steer != 0:
                        self._send_stop()

            # Send if needed
            if self._should_send(cmd):
                self._send_cmd(cmd)

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)


# ----------------------------
# Adapter: your existing pygame XboxController -> ControllerState
# ----------------------------
class XboxControllerAdapter(IController):
    """
    Adapter around your pygame-based XboxController class.
    Normalizes conventions:
      - forward is +1.0
      - turn right is +1.0
      - triggers become 0..1 (best-effort)
    """
    def __init__(self, ctrl: "XboxController") -> None:
        self.ctrl = ctrl

    def read_state(self) -> ControllerState:
        st = self.ctrl.read()

        # Pygame convention: LY up is -1.0 -> forward should be +1.0
        ly_raw = float(st.axes.get("LY", 0.0))
        lx_raw = float(st.axes.get("LX", 0.0))

        forward = clamp(-ly_raw, -1.0, 1.0)  # invert
        turn = clamp(lx_raw, -1.0, 1.0)

        # Triggers:
        # Depending on mapping, you might get LT/RT in [-1..1] or [0..1] or shared axis.
        # Here we attempt to normalize to 0..1 if they exist.
        lt_raw = float(st.axes.get("LT", 0.0))
        rt_raw = float(st.axes.get("RT", 0.0))

        def norm_trigger(x: float) -> float:
            # If it's already 0..1, keep it. If it's -1..1, map to 0..1.
            if -1.01 <= x <= 1.01:
                # Heuristic: if negative values appear, treat as -1..1
                if x < 0.0:
                    return (x + 1.0) * 0.5
                # If never negative, assume 0..1 already
                return clamp(x, 0.0, 1.0)
            return clamp(x, 0.0, 1.0)

        lt = norm_trigger(lt_raw)
        rt = norm_trigger(rt_raw)

        def btn(name: str) -> bool:
            return bool(st.buttons.get(name, 0))

        return ControllerState(
            forward=forward,
            turn=turn,
            lt=lt,
            rt=rt,
            a=btn("A"),
            b=btn("B"),
            x=btn("X"),
            y=btn("Y"),
            lb=btn("LB"),
            rb=btn("RB"),
            menu=btn("MENU"),
            view=btn("VIEW"),
        )


# ----------------------------
# Example integration (not a CLI; edit globals)
# ----------------------------
"""
Usage pattern (in your own runner file/module):

1) Create serial + protocol
2) Create controller + adapter
3) Create mapper + bridge
4) Run

Keep this in a function or separate module in your project if you prefer.
"""

# These are intentionally "globals you can edit" as requested:
DRIVE_HZ = 50.0
DRIVE_DEADZONE = 0.12
DRIVE_EXPO = 0.35
DRIVE_MAX_THR = 100 #max 100
DRIVE_MAX_STEER = 100
DRIVE_MS = 200

FAILSAFE_STOP_ON_INACTIVE_S = 2.0  # None disables


def build_drive_bridge(ctrl: "XboxController", proto: "NdjsonSerialProtocol") -> DriveBridge:
    tuning = DriveTuning(
        deadzone=DRIVE_DEADZONE,
        expo=DRIVE_EXPO,
        max_thr=DRIVE_MAX_THR,
        max_steer=DRIVE_MAX_STEER,
    )
    mapper = ArcadeDriveMapper(tuning)
    adapter = XboxControllerAdapter(ctrl)

    cfg = BridgeConfig(
        hz=DRIVE_HZ,
        stop_on_disconnect=True,
        stop_on_inactive_s=FAILSAFE_STOP_ON_INACTIVE_S,
        drv_ms=DRIVE_MS,
    )
    return DriveBridge(adapter, mapper, proto, cfg=cfg, tuning=tuning)


########################3 testing###############3
class PrintOnlyProtocol:
    """
    Drop-in stand-in for NdjsonSerialProtocol.
    Prints JSON objects instead of writing to serial.
    """
    def __init__(self) -> None:
        self._seq = 1

    def next_seq(self) -> int:
        s = self._seq
        self._seq += 1
        return s

    def send_drv(self, thr: int, steer: int = 0, *, ms: int = 200, src: str = "xbox") -> int:
        seq = self.next_seq()
        obj = {
            "v": 1,
            "type": "drv",
            "thr": thr,
            "str": steer,
            "ms": ms,
            "src": src,
            "seq": seq,
        }
        print(json.dumps(obj))
        return seq

    def send_ping(self, t_ms: int) -> int:
        seq = self.next_seq()
        print(json.dumps({"type": "ping", "seq": seq, "t": t_ms}))
        return seq




"""
Future extension ideas:
- Add a ModeManager: map buttons to proto.send_mode("...") and/or safety toggles.
- Add a 'turbo' modifier (RB) that increases max_thr/max_steer temporarily.
- Add a different mapper (tank drive using both sticks).
- Add ack tracking: keep a rolling window of seq numbers and last ack time.
- Add "soft-start" ramping: limit accel per second to avoid wheel slip.
"""