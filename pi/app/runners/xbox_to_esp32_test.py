# xbox_to_esp32.py
#
# Reads Xbox controller input and sends NDJSON drive ("drv") messages to ESP32
# over USB serial, using your NdjsonSerialProtocol.
#
# Requirements:
#   pip install pyserial pygame
#
# Run:
#   python3 xbox_to_esp32.py
#
# Notes:
# - Update SERIAL_PORT if needed (or set env SERIAL_PORT=/dev/ttyACM0, etc.)
# - Make sure ESP32 is flashed with the protocol listener and baud matches.

import os
import time
import serial

from app.controllers.xbox_controller import XboxController
from app.drive.xbox_drive_bridge import build_drive_bridge
from app.protocol.com_proto import NdjsonSerialProtocol  # adjust import path to wherever you placed it
#from app.utils import now_ms


# ----------------------------
# EDIT THESE (globals)
# ----------------------------
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyUSB0")
BAUD = int(os.getenv("SERIAL_BAUD", "115200"))
SERIAL_TIMEOUT_S = float(os.getenv("SERIAL_TIMEOUT_S", "0.05"))

LOG_TX = True
LOG_RX = True

HELLO_ON_START = True
HELLO_NAME = "pi-brain"


def main() -> None:
    print("Starting Xbox → ESP32 drive bridge")
    print(f"Serial: {SERIAL_PORT} @ {BAUD}")
    print("Move left stick to drive. Ctrl+C to exit.\n")

    # 1) Controller (pygame)
    ctrl = XboxController(index=0)

    # 2) Serial + protocol
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD,
        timeout=SERIAL_TIMEOUT_S,
        write_timeout=1.0,
    )

    # Small settle time after opening port (some USB serial adapters/ESP32s reset)
    time.sleep(0.4)

    proto = NdjsonSerialProtocol(ser, log_tx=LOG_TX, log_rx=LOG_RX)

    # Drain any boot noise so our first reads are clean
    proto.drain(duration_s=0.8)

    # Optional hello handshake (nice for sanity)
    if HELLO_ON_START:
        seq = proto.send_hello(name=HELLO_NAME)
        # Non-fatal if no response; still useful when ESP32 is speaking
        _ = proto.wait_for_type("hello_ack", seq=seq, timeout_s=1.2)

    # 3) Bridge: maps controller -> DriveCmd -> proto.send_drv(...)
    bridge = build_drive_bridge(ctrl, proto)

    try:
        bridge.run_forever()
    except KeyboardInterrupt:
        print("\nStopping... sending stop command.")
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