from __future__ import annotations
import time
from dataclasses import dataclass

from app.drive.types import DriveCmd, DriveTuning


@dataclass
class SendDecider:
    tuning: DriveTuning
    last_send_t: float = 0.0
    last_cmd: DriveCmd | None = None

    def should_send(self, cmd: DriveCmd) -> bool:
        now = time.time()

        if now - self.last_send_t < self.tuning.min_send_interval_s:
            return False

        if self.last_cmd is None:
            return True

        if (now - self.last_send_t) >= self.tuning.max_stale_s:
            return True

        if abs(cmd.thr - self.last_cmd.thr) >= self.tuning.min_change:
            return True
        if abs(cmd.steer - self.last_cmd.steer) >= self.tuning.min_change:
            return True

        return False

    def mark_sent(self, cmd: DriveCmd) -> None:
        self.last_cmd = cmd
        self.last_send_t = time.time()


@dataclass
class SlewLimiter:
    tuning: DriveTuning
    thr_cur: float = 0.0
    steer_cur: float = 0.0
    last_t: float = time.time()

    def apply(self, target: DriveCmd) -> DriveCmd:
        if not self.tuning.enable_slew:
            return target

        now = time.time()
        dt = max(0.0, now - self.last_t)
        self.last_t = now

        def slew(cur: float, tgt: float, accel: float, decel: float) -> float:
            # Accelerating if magnitude increases; otherwise decel
            rate = accel if abs(tgt) > abs(cur) else decel
            step = rate * dt
            if tgt > cur:
                return min(tgt, cur + step)
            return max(tgt, cur - step)

        self.thr_cur = slew(self.thr_cur, float(target.thr), self.tuning.thr_accel_per_s, self.tuning.thr_decel_per_s)

        # steering usually safe to slew symmetrically
        steer_step = self.tuning.steer_slew_per_s * dt
        if float(target.steer) > self.steer_cur:
            self.steer_cur = min(float(target.steer), self.steer_cur + steer_step)
        else:
            self.steer_cur = max(float(target.steer), self.steer_cur - steer_step)

        return DriveCmd(
            thr=int(round(self.thr_cur)),
            steer=int(round(self.steer_cur)),
            ms=target.ms,
            src=target.src,
        )