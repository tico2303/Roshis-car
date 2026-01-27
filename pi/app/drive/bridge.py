from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Optional, Protocol

from app.controllers.types import Controller
from app.utils import now_ms
from app.drive.types import BridgeConfig, DriveCmd, DriveTuning
from app.drive.mapping import ArcadeDriveMapper
from app.drive.filters import SendDecider, SlewLimiter


class DriveProtocol(Protocol):
    def send_drv(self, thr: int, steer: int = 0, *, ms: int = 200, src: str = "xbox") -> int: ...
    def send_ping(self, t_ms: int) -> int: ...


@dataclass
class DriveBridge:
    controller: Controller
    mapper: ArcadeDriveMapper
    proto: DriveProtocol
    cfg: BridgeConfig
    tuning: DriveTuning

    def run_forever(self) -> None:
        period = 1.0 / max(1.0, self.cfg.hz)

        decider = SendDecider(self.tuning)
        slew = SlewLimiter(self.tuning)
        last_activity_t = time.time()
        last_heartbeat_t = time.time()

        while True:
            loop_start = time.time()

            # Heartbeat
            if self.cfg.heartbeat_s is not None and (time.time() - last_heartbeat_t) >= self.cfg.heartbeat_s:
                self.proto.send_ping(now_ms())
                last_heartbeat_t = time.time()

            # Read controller
            try:
                st = self.controller.read_state()
            except Exception:
                if self.cfg.stop_on_disconnect:
                    self.proto.send_drv(0, 0, ms=self.cfg.drv_ms, src="failsafe")
                raise

            # Map + filter
            cmd = self.mapper.map(st)
            cmd = DriveCmd(thr=cmd.thr, steer=cmd.steer, ms=self.cfg.drv_ms, src=cmd.src)
            cmd = slew.apply(cmd)

            # Activity tracking
            if decider.last_cmd is None or (cmd.thr != decider.last_cmd.thr or cmd.steer != decider.last_cmd.steer):
                last_activity_t = time.time()

            # Inactivity failsafe (optional)
            if self.cfg.stop_on_inactive_s is not None:
                if (time.time() - last_activity_t) >= self.cfg.stop_on_inactive_s:
                    if decider.last_cmd is None or decider.last_cmd.thr != 0 or decider.last_cmd.steer != 0:
                        cmd = DriveCmd(thr=0, steer=0, ms=self.cfg.drv_ms, src="failsafe")

            # Send
            if decider.should_send(cmd):
                self.proto.send_drv(cmd.thr, cmd.steer, ms=cmd.ms, src=cmd.src)
                decider.mark_sent(cmd)

            # Maintain rate
            elapsed = time.time() - loop_start
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)