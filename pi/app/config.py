from __future__ import annotations

import os


def env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "y", "on")


def env_int(name: str, default: int) -> int:
    v = os.getenv(name)
    return default if v is None else int(v)


def env_float(name: str, default: float) -> float:
    v = os.getenv(name)
    return default if v is None else float(v)


def env_str(name: str, default: str) -> str:
    v = os.getenv(name)
    return default if v is None else v


# ============================
# Serial / Protocol
# ============================
SERIAL_PORT = env_str("SERIAL_PORT", "/dev/ttyUSB0")  # often /dev/ttyACM0 on ESP32
SERIAL_BAUD = env_int("SERIAL_BAUD", 460800)
SERIAL_TIMEOUT_S = env_float("SERIAL_TIMEOUT_S", 0.05)

LOG_TX = env_bool("LOG_TX", True)
LOG_RX = env_bool("LOG_RX", True)

SEND_HELLO = env_bool("SEND_HELLO", True)
HELLO_NAME = env_str("HELLO_NAME", "pi-brain")

# ============================
# MOTOR Controller
# ============================
CTRL_INDEX = env_int("CTRL_INDEX", 0)
CTRL_DEADZONE = env_float("CTRL_DEADZONE", 0.12)

# ============================
# MOTOR Bridge loop behavior
# ============================
DRIVE_HZ = env_float("DRIVE_HZ", 50.0)
DRIVE_MS = env_int("DRIVE_MS", 250)
STOP_ON_INACTIVE_S = env_float("STOP_ON_INACTIVE_S", 2.0)

# ============================
# MOTOR Mapping / Tuning
# ============================
DRIVE_EXPO = env_float("DRIVE_EXPO", 0.35)
MAX_THR = env_int("MAX_THR", 80)       # start conservative to avoid PSU OCP
MAX_STEER = env_int("MAX_STEER", 100)

MIN_SEND_INTERVAL_S = env_float("MIN_SEND_INTERVAL_S", 0.05)
MAX_STALE_S = env_float("MAX_STALE_S", 0.15)
MIN_CHANGE = env_int("MIN_CHANGE", 2)

# Slew limiting (optional)
ENABLE_SLEW = env_bool("ENABLE_SLEW", True)

THR_ACCEL_PER_S = env_float("THR_ACCEL_PER_S", 90.0)    # gentle ramp up
THR_DECEL_PER_S = env_float("THR_DECEL_PER_S", 800.0)   # fast stop
STEER_SLEW_PER_S = env_float("STEER_SLEW_PER_S", 250.0)