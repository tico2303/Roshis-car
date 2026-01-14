import json
import time
import serial
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple

from xbox_controller import XboxController  # your existing module

PORT = "/dev/ttyUSB0"
BAUD = 115200


# ----------------------------
# Utilities
# ----------------------------
def now_ms() -> int:
    return int(time.time() * 1000)


def clamp(n: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, n))


def apply_deadzone(x: float, deadzone: float) -> float:
    return 0.0 if abs(x) < deadzone else x


def scale_to_int100(x: float) -> int:
    """
    Convert -1.0..1.0 -> -100..100
    """
    return int(round(clamp(x, -1.0, 1.0) * 100))


def _escape_bytes(b: bytes) -> str:
    # Makes control chars visible
    return b.decode("utf-8", "backslashreplace").replace("\n", "\\n").replace("\r", "\\r")


# ----------------------------
# Protocol layer (NDJSON over USB serial)
# ----------------------------
class NdjsonSerialProtocol:
    def __init__(self, ser: serial.Serial, *, log_tx: bool = True, log_rx: bool = True):
        self.ser = ser
        self.log_tx = log_tx
        self.log_rx = log_rx
        self._seq = 1

    def next_seq(self) -> int:
        s = self._seq
        self._seq += 1
        return s

    def send(self, obj: Dict[str, Any]) -> None:
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        raw = line.encode("utf-8")

        if self.log_tx:
            print("TX_RAW :", _escape_bytes(raw))

        self.ser.write(raw)
        self.ser.flush()

    def recv(self, timeout_s: float = 0.05) -> Optional[Dict[str, Any]]:
        self.ser.timeout = timeout_s
        raw = self.ser.readline()
        if not raw:
            return None

        if self.log_rx:
            print("RX_RAW :", _escape_bytes(raw))

        s = raw.decode("utf-8", "backslashreplace").strip()
        try:
            return json.loads(s)
        except Exception as e:
            # Keep non-fatal; bad boot fragments happen.
            if self.log_rx:
                print("RX_ERR :", repr(e))
            return {"type": "err", "msg": "pi_json_decode_failed", "raw": s}

    def drain(self, duration_s: float = 1.0) -> None:
        start = time.time()
        while time.time() - start < duration_s:
            _ = self.recv(timeout_s=0.05)

    # Convenience message builders
    def send_hello(self, name: str = "pi-brain") -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "hello", "name": name, "proto": "ndjson", "seq": seq, "ts": now_ms()})
        return seq

    def send_ping(self, t_ms: int) -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "ping", "t": t_ms, "seq": seq, "ts": now_ms()})
        return seq

    def send_mode(self, mode_name: str) -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "mode", "name": mode_name, "seq": seq, "ts": now_ms()})
        return seq

    def send_drv(self, thr: int, steer: int = 0, *, ms: int = 200, src: str = "xbox") -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "drv", "thr": thr, "str": steer, "ms": ms, "src": src, "seq": seq, "ts": now_ms()})
        return seq


# ----------------------------
# Controller abstraction
# ----------------------------
@dataclass(frozen=True)
class ControllerState:
    """
    Normalized controller state.
    Axes: floats in [-1.0, 1.0]
    Buttons: bool
    """
    ly: float  # left stick Y (forward/back) convention: up is -1 on many libs
    a: bool
    b: bool
    x: bool
    y: bool
    lb: bool
    rb: bool
    start: bool
    back: bool


class XboxControllerAdapter:
    """
    Adapter around your XboxController class.
    You MUST edit `read_state()` to match your XboxController API.
    """
    def __init__(self) -> None:
        self.ctrl = XboxController()

    def read_state(self) -> ControllerState:
        """
        TODO: Modify this to match your XboxController implementation.
        Below are placeholder attribute names.
        """
        # Common patterns (examples):
        # - self.ctrl.ly
        # - self.ctrl.get_left_stick_y()
        # - self.ctrl.state["ly"]
        #
        # Replace these with your real accessors:
        ly = float(getattr(self.ctrl, "ly", 0.0))

        def btn(name: str) -> bool:
            return bool(getattr(self.ctrl, name, False))

        return ControllerState(
            ly=ly,
            a=btn("a"),
            b=btn("b"),
            x=btn("x"),
            y=btn("y"),
            lb=btn("lb"),
            rb=btn("rb"),
            start=btn("start"),
            back=btn("back"),
        )


# ----------------------------
# Input mapping + actions
# ----------------------------
@dataclass
class DriveIntent:
    thr: int  # -100..100
    steer: int = 0  # keep for future


class EdgeDetector:
    """
    Tracks button rising edges so actions fire once per press.
    """
    def __init__(self) -> None:
        self._prev: Dict[str, bool] = {}

    def rising(self, key: str, current: bool) -> bool:
        prev = self._prev.get(key, False)
        self._prev[key] = current
        return (not prev) and current


class ActionBindings:
    """
    Bind buttons to actions (extensible).
    Actions are callables that take (app_context) or no args.
    """
    def __init__(self) -> None:
        self._bindings: Dict[str, Callable[[], None]] = {}

    def bind(self, name: str, action: Callable[[], None]) -> None:
        self._bindings[name] = action

    def run(self, name: str) -> None:
        action = self._bindings.get(name)
        if action:
            action()


# ----------------------------
# Main App
# ----------------------------
class CarBrainApp:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud

        self.controller = XboxControllerAdapter()
        self.edges = EdgeDetector()

        self.enabled_drive = False  # "arming" switch
        self.speed_limit = 1.0  # scale 0..1 (LB/RB can change later)
        self.current_mode = "confident"

        # bindings set in run()
        self.bindings = ActionBindings()

    def _compute_throttle_forward_reverse_only(self, ly: float) -> int:
        """
        Forward/reverse only for now:
        - Use left stick Y as throttle.
        - Many libs report up as -1, down as +1, so we invert.
        """
        ly = apply_deadzone(ly, deadzone=0.10)
        thr = scale_to_int100(-ly)  # invert so stick up => forward (+)
        thr = int(thr * self.speed_limit)
        return int(clamp(thr, -100, 100))

    def run(self) -> None:
        with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
            time.sleep(0.8)  # allow ESP32 boot/reset on port open
            proto = NdjsonSerialProtocol(ser, log_tx=True, log_rx=True)

            # Drain boot noise
            proto.drain(duration_s=0.8)

            # Handshake + optional ping test (kept lightweight)
            proto.send_hello()
            _ = proto.recv(timeout_s=1.5)

            t0 = now_ms()
            proto.send_ping(t0)
            pong = proto.recv(timeout_s=1.5)
            if pong and pong.get("type") == "pong" and pong.get("t") == t0:
                print(f"Ping OK RTT≈{now_ms() - t0}ms")

            # ---- Bind buttons to actions (extensible) ----
            # A: Arm/Disarm driving
            self.bindings.bind("A", lambda: self._toggle_drive(proto))
            # B: Emergency stop (also disarm)
            self.bindings.bind("B", lambda: self._estop(proto))
            # X: Toggle drama mode
            self.bindings.bind("X", lambda: self._set_mode(proto, "drama"))
            # Y: Toggle reflective mode
            self.bindings.bind("Y", lambda: self._set_mode(proto, "reflective"))
            # Start: Arrival macro placeholder (no motors yet, but sets mode)
            self.bindings.bind("START", lambda: self._set_mode(proto, "arrival"))

            print("Running. Press A to arm. B for E-STOP. Ctrl+C to quit.")

            # Main loop (controller → intent → protocol)
            loop_hz = 50
            dt = 1.0 / loop_hz

            try:
                while True:
                    state = self.controller.read_state()

                    # Rising-edge actions
                    if self.edges.rising("a", state.a):
                        self.bindings.run("A")
                    if self.edges.rising("b", state.b):
                        self.bindings.run("B")
                    if self.edges.rising("x", state.x):
                        self.bindings.run("X")
                    if self.edges.rising("y", state.y):
                        self.bindings.run("Y")
                    if self.edges.rising("start", state.start):
                        self.bindings.run("START")

                    # Drive stream
                    if self.enabled_drive:
                        thr = self._compute_throttle_forward_reverse_only(state.ly)
                        proto.send_drv(thr=thr, steer=0, ms=200, src="xbox")
                    else:
                        # When disarmed, optionally keep sending neutral at low rate or not at all.
                        # Sending neutral helps future watchdog behavior on ESP32.
                        proto.send_drv(thr=0, steer=0, ms=200, src="xbox")

                    # Read any incoming messages (non-blocking)
                    msg = proto.recv(timeout_s=0.01)
                    if msg and msg.get("type") not in (None, "ack"):
                        # keep acks quiet; print other messages
                        print("RX:", msg)

                    time.sleep(dt)
            except KeyboardInterrupt:
                self._estop(proto)
                print("\nStopped.")

    # ---- Actions ----
    def _toggle_drive(self, proto: NdjsonSerialProtocol) -> None:
        self.enabled_drive = not self.enabled_drive
        mode = "armed" if self.enabled_drive else "disarmed"
        print(f"Drive {mode.upper()}")
        # Optionally send a mode so ESP32 can reflect LEDs later
        proto.send_mode(mode)

    def _estop(self, proto: NdjsonSerialProtocol) -> None:
        print("E-STOP")
        self.enabled_drive = False
        proto.send_mode("stop")
        proto.send_drv(thr=0, steer=0, ms=200, src="xbox")

    def _set_mode(self, proto: NdjsonSerialProtocol, name: str) -> None:
        # Toggle behavior for drama/reflective if you want
        if name in ("drama", "reflective") and self.current_mode == name:
            name = "confident"
        self.current_mode = name
        print(f"MODE -> {name}")
        proto.send_mode(name)


def main() -> None:
    app = CarBrainApp(PORT, BAUD)
    app.run()


if __name__ == "__main__":
    main()
