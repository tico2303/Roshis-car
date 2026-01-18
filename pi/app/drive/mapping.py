from __future__ import annotations
from dataclasses import dataclass

from app.utils import clamp, apply_deadzone, scale_to_int100
from app.controllers.types import ControllerState
from app.drive.types import DriveCmd, DriveTuning


def expo_curve(x: float, expo: float) -> float:
    x = clamp(x, -1.0, 1.0)
    return (1.0 - expo) * x + expo * (x * x * x)


@dataclass
class ArcadeDriveMapper:
    tuning: DriveTuning

    def map(self, st: ControllerState) -> DriveCmd:
        f = expo_curve(apply_deadzone(st.forward, self.tuning.deadzone), self.tuning.expo)
        t = expo_curve(apply_deadzone(st.turn, self.tuning.deadzone), self.tuning.expo)

        thr = int(round(scale_to_int100(f)))
        steer = int(round(scale_to_int100(t)))

        thr = int(clamp(thr, -self.tuning.max_thr, self.tuning.max_thr))
        steer = int(clamp(steer, -self.tuning.max_steer, self.tuning.max_steer))

        return DriveCmd(thr=thr, steer=steer)