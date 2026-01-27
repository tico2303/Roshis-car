"""
CarBrainApp - single canonical runner for the Pi brain.

This file is intended to be the one and only entry point for driving logic
and controller-to-ESP32 protocol glue. The repository previously contained
multiple runners (`app.py` at project root and `app/app.py`) with largely
duplicated code. Those duplicates were removed and this module now contains
the consolidated, commented implementation.

How to run:
- From the project root: `python3 -m app.main` or `python3 app/main.py`

High-level responsibilities:
- Read controller state via `XboxControllerAdapter`.
- Map button presses to actions using `InputFacade`.
- Compute drive intents (throttle/steer) from joystick inputs.
- Send NDJSON messages to the ESP32 via `NdjsonSerialProtocol`.

Notes on imports and execution:
- This module uses local (package-root) imports so it runs when executed as
  `python app/main.py` (Python adds `app/` to `sys.path` in that case). If you
  prefer strict package-relative imports, run with `python -m app.main`.
"""

import time
import serial
from dataclasses import dataclass

# Local modules (kept as simple imports so `python app/main.py` works)
from xbox_controller import XboxControllerAdapter
from com_proto import NdjsonSerialProtocol
from utils import now_ms, clamp, apply_deadzone, scale_to_int100
from input_helpers import InputFacade


# Serial port defaults (override by editing or env/wrapping runner)
PORT = "/dev/ttyUSB0"
BAUD = 115200


# ---------- Small data model for intent ----------
@dataclass
class DriveIntent:
    thr: int  # -100..100
    steer: int = 0


# ---------- Main application class ----------
class CarBrainApp:
    """Main controller that bridges controller -> serial protocol.

    Responsibilities:
    - Initialize controller and input facade
    - Maintain simple state (armed/disarmed, current mode, speed limit)
    - Run main loop that reads controller, maps to commands, and sends NDJSON
    """

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud

        # Controller adapter provides `read_state()` returning a small state
        # object with attributes like `a`, `b`, `x`, `y`, `start`, `ly` etc.
        self.controller = XboxControllerAdapter()
        # InputFacade handles rising-edge detection and bound callbacks
        self.input = InputFacade()

        # runtime flags
        self.enabled_drive = False  # arm/disarm
        self.speed_limit = 1.0  # scale factor 0..1
        self.current_mode = "confident"

    def _compute_throttle_forward_reverse_only(self, ly: float) -> int:
        """Compute a single throttle value from left-stick Y.

        The left stick is treated as a forward/reverse control. We apply a
        small deadzone, invert the axis if necessary, scale to -100..100 and
        apply `speed_limit`.
        """
        ly = apply_deadzone(ly, deadzone=0.10)
        thr = scale_to_int100(-ly)  # invert: stick-up => positive throttle
        thr = int(thr * self.speed_limit)
        return int(clamp(thr, -100, 100))

    def run(self) -> None:
        """Open serial, handshake, bind buttons, and run main control loop."""
        with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
            time.sleep(0.8)  # allow ESP32 boot/reset when opening port
            proto = NdjsonSerialProtocol(ser, log_tx=True, log_rx=True)

            # Drain any boot noise printed by the microcontroller
            proto.drain(duration_s=0.8)

            # Lightweight handshake and optional ping latency check
            proto.send_hello()
            _ = proto.recv(timeout_s=1.5)

            t0 = now_ms()
            proto.send_ping(t0)
            pong = proto.recv(timeout_s=1.5)
            if pong and pong.get("type") == "pong" and pong.get("t") == t0:
                print(f"Ping OK RTT≈{now_ms() - t0}ms")

            # ---- Bind controller buttons to actions ----
            # A: Arm/Disarm
            self.input.bind("A", lambda: self._toggle_drive(proto))
            # B: E-Stop (also disarm)
            self.input.bind("B", lambda: self._estop(proto))
            # Mode buttons (toggle-ish behavior)
            self.input.bind("X", lambda: self._set_mode(proto, "drama"))
            self.input.bind("Y", lambda: self._set_mode(proto, "reflective"))
            self.input.bind("START", lambda: self._set_mode(proto, "arrival"))

            print("Running. Press A to arm. B for E-STOP. Ctrl+C to quit.")

            loop_hz = 50
            dt = 1.0 / loop_hz

            try:
                while True:
                    state = self.controller.read_state()

                    # Process rising-edge button events via InputFacade
                    self.input.process("a", state.a)
                    self.input.process("b", state.b)
                    self.input.process("x", state.x)
                    self.input.process("y", state.y)
                    self.input.process("start", state.start)

                    # Build and send drive commands depending on armed state
                    if self.enabled_drive:
                        thr = self._compute_throttle_forward_reverse_only(state.ly)
                        proto.send_drv(thr=thr, steer=0, ms=200, src="xbox")
                    else:
                        # Keep sending neutral so remote watchdogs can behave
                        proto.send_drv(thr=0, steer=0, ms=200, src="xbox")

                    # Non-blocking read for messages from ESP32
                    msg = proto.recv(timeout_s=0.01)
                    if msg and msg.get("type") not in (None, "ack"):
                        print("RX:", msg)

                    time.sleep(dt)
            except KeyboardInterrupt:
                # Ensure motors are stopped on exit
                self._estop(proto)
                print("\nStopped.")

    # ---- Action helpers ----
    def _toggle_drive(self, proto: NdjsonSerialProtocol) -> None:
        self.enabled_drive = not self.enabled_drive
        mode = "armed" if self.enabled_drive else "disarmed"
        print(f"Drive {mode.upper()}")
        proto.send_mode(mode)

    def _estop(self, proto: NdjsonSerialProtocol) -> None:
        print("E-STOP")
        self.enabled_drive = False
        proto.send_mode("stop")
        proto.send_drv(thr=0, steer=0, ms=200, src="xbox")

    def _set_mode(self, proto: NdjsonSerialProtocol, name: str) -> None:
        # Toggle back to confident if same mode pressed twice for these modes
        if name in ("drama", "reflective") and self.current_mode == name:
            name = "confident"
        self.current_mode = name
        print(f"MODE -> {name}")
        proto.send_mode(name)


def main() -> None:
    app = CarBrainApp(PORT, BAUD)
    app.run()


if __name__ == "__main__":
    import os
    print("===== Main.py Started =====")
    print("PID:", os.getpid())
    main()
