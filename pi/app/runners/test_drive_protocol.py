from __future__ import annotations
from app.protocol.com_proto import NdjsonSerialProtocol
import json
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple
import serial
from app.utils import _escape_bytes, now_ms, guess_esp32_port, list_serial_candidates

"""
Simple "edit-the-variables" script:
- Opens serial
- (optionally) drains boot noise
- (optionally) hello + ping
- Sends a list of DRV commands
- Prints every RX line + parsed object
"""

# ----------------------------
# Config you edit
# ----------------------------
SERIAL_PORT = "/dev/ttyUSB0"     # e.g. "/dev/tty.usbserial-0001" or "COM5"
BAUD = 115200

LOG_TX = True
LOG_RX = True

DRAIN_ON_CONNECT_S = 0.6
SEND_HELLO = True
SEND_PING = False

HELLO_NAME = "pi-brain"
PING_TIMEOUT_S = 2.0

ACK_TIMEOUT_S = 1.5
INTERVAL_BETWEEN_CMDS_S = 0.25

# Each command is (thr, steer, ms, src). You can edit this list freely.
DRV_COMMANDS: Sequence[Tuple[int, int, int, str]] = [
    (50, 0, 200, "pi-script"),
    (50, -40, 200, "pi-script"),
    (50, -40, 200, "pi-script"),
    (50, 40, 200, "pi-script"),
    (50, 40, 200, "pi-script"),
    (0, 0, 200, "pi-script"),
]
SERIAL_PORT = "/dev/ttyUSB0"  # optional fallback; can be None
BAUD = 115200


def open_serial() -> serial.Serial:
    try:
        port = guess_esp32_port(preferred=SERIAL_PORT, wait_s=3.0)
    except FileNotFoundError as e:
        print("ERROR :", e)
        print("DEBUG : candidates =", list_serial_candidates())
        raise
    print(f"INFO  : using serial port {port}")
    ser = serial.Serial(port, BAUD, timeout=0.1)

    # Optional: reduce “reset on open” behavior on some boards
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    # Give the board time if it rebooted
    time.sleep(0.2)
    return ser


# ----------------------------
# Runner
# ----------------------------
def run() -> None:
    with open_serial() as ser:
        proto = NdjsonSerialProtocol(ser, log_tx=LOG_TX, log_rx=LOG_RX)

        # Many ESP32 boards reset on serial open; give it a moment.
        time.sleep(0.3)

        if DRAIN_ON_CONNECT_S > 0:
            proto.drain(DRAIN_ON_CONNECT_S)

        if SEND_HELLO:
            hello_seq = proto.send_hello(HELLO_NAME)
            hello_ack = proto.wait_for_type("hello_ack", seq=hello_seq, timeout_s=2.0)
            if hello_ack:
                print("INFO  : hello_ack received")
            else:
                print("WARN  : hello_ack not received")

        if SEND_PING:
            ping_seq = proto.send_ping(now_ms())
            pong = proto.wait_for_type("pong", seq=ping_seq, timeout_s=PING_TIMEOUT_S)
            if pong:
                print("INFO  : pong received:", pong)
            else:
                print("WARN  : pong not received")

        for thr, steer, ms, src in DRV_COMMANDS:
            seq = proto.send_drv(thr=thr, steer=steer, ms=ms, src=src)
            ok = proto.wait_for_ack(seq=seq, ack_type="drv", timeout_s=ACK_TIMEOUT_S)
            if not ok:
                print(f"WARN  : no drv ack for seq={seq} (thr={thr}, str={steer})")

            time.sleep(INTERVAL_BETWEEN_CMDS_S)

        # Drain anything left
        proto.drain(0.4)


if __name__ == "__main__":
    run()