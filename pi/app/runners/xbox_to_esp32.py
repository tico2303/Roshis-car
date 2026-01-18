from __future__ import annotations

import time
import serial

from app import config
from app.controllers.xbox_pygame import XboxController, XboxPygameAdapter
from app.drive.types import DriveTuning, BridgeConfig
from app.drive.mapping import ArcadeDriveMapper
from app.drive.bridge import DriveBridge
from app.protocol.ndjson_serial import NdjsonSerialProtocol


def main() -> None:
    print("Xbox → ESP32 bridge starting")
    print(f"Serial: {config.SERIAL_PORT} @ {config.SERIAL_BAUD}")
    print("Ctrl+C to exit.\n")

    # 1) Controller
    raw = XboxController(index=config.CTRL_INDEX, deadzone=config.CTRL_DEADZONE)
    controller = XboxPygameAdapter(raw)

    # 2) Serial + protocol
    ser = serial.Serial(
        port=config.SERIAL_PORT,
        baudrate=config.SERIAL_BAUD,
        timeout=config.SERIAL_TIMEOUT_S,
        write_timeout=1.0,
    )

    # Many ESP32 boards reset when serial opens; settle time helps
    time.sleep(0.5)

    proto = NdjsonSerialProtocol(ser, log_tx=config.LOG_TX, log_rx=config.LOG_RX)

    proto.drain(duration_s=0.8)

    if config.SEND_HELLO:
        try:
            seq = proto.send_hello(name=config.HELLO_NAME)
            _ = proto.wait_for_type("hello_ack", seq=seq, timeout_s=1.2)
        except Exception as e:
            print(f"[warn] hello/ack not confirmed: {e!r}")

    # 3) Bridge
    tuning = DriveTuning(
        deadzone=config.CTRL_DEADZONE,
        expo=config.DRIVE_EXPO,
        max_thr=config.MAX_THR,
        max_steer=config.MAX_STEER,
        min_send_interval_s=config.MIN_SEND_INTERVAL_S,
        max_stale_s=config.MAX_STALE_S,
        min_change=config.MIN_CHANGE,
        enable_slew=config.ENABLE_SLEW,
        thr_accel_per_s=config.THR_ACCEL_PER_S,
        thr_decel_per_s=config.THR_DECEL_PER_S,
        steer_slew_per_s=config.STEER_SLEW_PER_S,
    )

    cfg = BridgeConfig(
        hz=config.DRIVE_HZ,
        drv_ms=config.DRIVE_MS,
        stop_on_disconnect=True,
        stop_on_inactive_s=config.STOP_ON_INACTIVE_S,
        heartbeat_s=None,
    )

    mapper = ArcadeDriveMapper(tuning)
    bridge = DriveBridge(controller, mapper, proto, cfg, tuning)

    try:
        bridge.run_forever()
    except KeyboardInterrupt:
        print("\nStopping. Sending stop command...")
        try:
            proto.send_drv(0, 0, ms=200, src="shutdown")
        except Exception:
            pass
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()