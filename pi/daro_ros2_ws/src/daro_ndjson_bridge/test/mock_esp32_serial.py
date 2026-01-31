#!/usr/bin/env python3
import json
import os
import pty
import select
import sys
import time


def write_line(fd: int, obj: dict) -> None:
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    os.write(fd, line.encode("utf-8"))


def main() -> None:
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    print(f"[mock-esp32] PTY slave device: {slave_name}")
    print("[mock-esp32] Run the ROS bridge with:")
    print(f"  ros2 run daro_ndjson_bridge ndjson_bridge --ros-args -p port:={slave_name} -p baud:=115200")
    print()

    # Make the PTY "raw-ish"
    os.set_blocking(master_fd, False)

    last_tx = 0.0
    buf = b""

    while True:
        now = time.time()

        # Periodically send a fake message "from ESP32" -> bridge RX
        if now - last_tx > 0.5:
            last_tx = now
            # Use whatever types you like; bridge will auto-create topics based on "type"
            write_line(master_fd, {"type": "hello", "proto": 1, "fw": "mock-esp32", "ts_ms": int(now * 1000)})
            write_line(master_fd, {"type": "mot", "l_pwm": 120, "r_pwm": 118, "ts_ms": int(now * 1000)})

        # Read anything written by bridge (bridge TX -> "ESP32")
        r, _, _ = select.select([master_fd], [], [], 0.05)
        if master_fd in r:
            try:
                chunk = os.read(master_fd, 4096)
            except BlockingIOError:
                chunk = b""

            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        msg = json.loads(line.decode("utf-8", errors="replace"))
                    except Exception:
                        print(f"[mock-esp32] RX (bad json): {line!r}")
                        continue

                    print(f"[mock-esp32] RX from bridge: {msg}")

                    # Optional: respond to drive commands with an ack
                    if msg.get("type") == "drv":
                        write_line(
                            master_fd,
                            {
                                "type": "drv_ack",
                                "ok": True,
                                "thr": msg.get("thr"),
                                "str": msg.get("str"),
                                "ts_ms": int(time.time() * 1000),
                            },
                        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[mock-esp32] exiting")
        sys.exit(0)