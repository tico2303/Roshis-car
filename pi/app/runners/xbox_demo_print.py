from app.controllers.xbox_pygame import XboxController, XboxPygameAdapter
from app.drive.types import DriveTuning, BridgeConfig
from app.drive.mapping import ArcadeDriveMapper
from app.drive.bridge import DriveBridge
from app.protocol.print_protocol import PrintOnlyProtocol


def main() -> None:
    ctrl = XboxController(index=0, deadzone=0.12)
    adapter = XboxPygameAdapter(ctrl)

    tuning = DriveTuning(
        deadzone=0.12,
        expo=0.35,
        max_thr=100,
        max_steer=100,
        max_stale_s=0.15,
        enable_slew=False,  # for demo prints, usually off
    )
    cfg = BridgeConfig(hz=50.0, drv_ms=250, stop_on_inactive_s=2.0)

    mapper = ArcadeDriveMapper(tuning)
    proto = PrintOnlyProtocol()

    bridge = DriveBridge(adapter, mapper, proto, cfg, tuning)
    bridge.run_forever()


if __name__ == "__main__":
    main()