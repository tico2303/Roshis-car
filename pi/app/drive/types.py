from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class DriveCmd:
    thr: int          # -100..100
    steer: int        # -100..100
    ms: int = 200
    src: str = "xbox"


@dataclass(frozen=True)
class DriveTuning:
    deadzone: float = 0.12
    expo: float = 0.35
    max_thr: int = 100
    max_steer: int = 100

    # send behavior
    min_send_interval_s: float = 0.05
    min_change: int = 2
    max_stale_s: float = 0.15

    # slew limiting (optional)
    enable_slew: bool = True
    thr_accel_per_s: float = 90.0
    thr_decel_per_s: float = 800.0
    steer_slew_per_s: float = 250.0


@dataclass(frozen=True)
class BridgeConfig:
    hz: float = 50.0
    stop_on_disconnect: bool = True
    stop_on_inactive_s: Optional[float] = 2.0
    heartbeat_s: Optional[float] = None
    drv_ms: int = 200