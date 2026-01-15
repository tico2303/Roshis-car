import time
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

