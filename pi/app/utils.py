import time
import glob
import os
from typing import Iterable, Optional
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


def _escape_bytes(b: bytes, max_len: int = 240) -> str:
    if len(b) > max_len:
        b = b[:max_len] + b"..."
    return b.decode("utf-8", "backslashreplace").replace("\n", "\\n").replace("\r", "\\r")

def list_serial_candidates() -> list[str]:
    """
    Return candidate serial device paths, ordered by preference.

    Preference:
      1) Stable symlinks: /dev/serial/by-id/*
      2) USB serial devices: /dev/ttyUSB*
      3) CDC ACM devices: /dev/ttyACM*
    """
    candidates: list[str] = []

    # (1) Stable names (Linux)
    candidates += sorted(glob.glob("/dev/serial/by-id/*"))

    # (2) Typical USB-serial bridges (CP210x, CH340, FTDI)
    candidates += sorted(glob.glob("/dev/ttyUSB*"))

    # (3) CDC ACM (some ESP32-S2/S3/C3 firmwares, some dev boards)
    candidates += sorted(glob.glob("/dev/ttyACM*"))

    # Filter to existing character devices / symlinks that point to them
    out: list[str] = []
    for p in candidates:
        try:
            if os.path.exists(p):
                out.append(p)
        except OSError:
            pass
    return out

def guess_esp32_port(preferred: Optional[str] = None, *, wait_s: float = 0.0) -> str:
    """
    Best-effort serial port selection for ESP32-like devices.

    - If 'preferred' exists, use it.
    - Otherwise scan known device patterns.
    - If wait_s > 0, keep scanning until timeout (helps if the device enumerates slowly).
    """
    if preferred and os.path.exists(preferred):
        return preferred

    deadline = time.time() + max(0.0, wait_s)
    while True:
        candidates = list_serial_candidates()
        if candidates:
            # Prefer by-id if present; otherwise first match in our ordered list.
            return candidates[0]
        if time.time() >= deadline:
            break
        time.sleep(0.1)

    raise FileNotFoundError(
        "No serial port found. Looked for /dev/serial/by-id/*, /dev/ttyUSB*, /dev/ttyACM*."
    )