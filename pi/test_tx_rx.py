#!/usr/bin/env python3
"""
TX/RX stress test — sends drv2 commands at configurable rate while
monitoring incoming COBS frames. No ROS2 needed.

Layers (set via command line):
  0: RX only, no TX           (baseline — should match test_serial_raw.py)
  1: TX 1 msg/sec             (slow — should be fine)
  2: TX 10 msg/sec            (moderate)
  3: TX 50 msg/sec            (matches real twist_to_drv rate)
  4: TX 50 msg/sec, single-threaded (TX and RX on same thread)

Usage: python3 test_tx_rx.py [layer] [port] [baud]
  Default: layer=0 /dev/ttyUSB0 115200

Flash ESP32 with main.cpp (COBS protocol) first.
"""

import json
import queue
import subprocess
import sys
import serial
import threading
import time


def crc16(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0:
            return b""
        for _ in range(code - 1):
            if i >= len(data):
                return b""
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = len(out)
    out.append(0)
    code = 1
    for b in data:
        if b != 0:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
        else:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
    out[code_idx] = code
    out.append(0x00)
    return bytes(out)


def make_drv2_frame(left: float = 0.0, right: float = 0.0) -> bytes:
    """Build a COBS-encoded drv2 command frame."""
    obj = {"type": "drv2", "left": round(left, 4), "right": round(right, 4)}
    compact = json.dumps(obj, separators=(",", ":"))
    crc = crc16(compact.encode("utf-8"))
    payload = f"{compact}\t{crc:04x}".encode("utf-8")
    return cobs_encode(payload)


TX_RATES = {
    0: 0,     # no TX
    1: 1,     # 1 Hz
    2: 10,    # 10 Hz
    3: 50,    # 50 Hz (real rate)
    4: 50,    # 50 Hz single-threaded
}


def tx_thread_fn(ser, rate_hz, stop_event, tx_stats):
    """Sends drv2 frames at the given rate on a separate thread."""
    if rate_hz <= 0:
        return
    period = 1.0 / rate_hz
    frame = make_drv2_frame(0.0, 0.0)
    while not stop_event.is_set():
        try:
            ser.write(frame)
            tx_stats["count"] += 1
        except Exception as e:
            tx_stats["errors"] += 1
        time.sleep(period)


def main():
    layer = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    rate_hz = TX_RATES.get(layer, 0)
    single_threaded = (layer == 4)

    print(f"=== TX/RX Stress Test ===")
    print(f"  Layer:  {layer}")
    print(f"  TX:     {rate_hz} Hz {'(single-threaded)' if single_threaded else '(separate thread)' if rate_hz > 0 else '(disabled)'}")
    print(f"  Port:   {port} @ {baud}")
    print()

    # Check port
    try:
        result = subprocess.run(["fuser", port], capture_output=True, text=True, timeout=3)
        pids = result.stdout.strip()
        if pids:
            print(f"  *** WARNING: {port} in use by PID(s): {pids} ***")
            print(f"  Kill with: sudo kill {pids.split()[-1]}")
            resp = input("  Continue? [y/N] ")
            if resp.lower() != "y":
                sys.exit(1)
    except Exception:
        pass

    ser = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=0.1)
    ser.dtr = False
    ser.rts = False
    ser.reset_input_buffer()
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Stats
    stats = {
        "rx_frames": 0, "cobs_ok": 0, "cobs_fail": 0,
        "crc_ok": 0, "crc_fail": 0, "crc_no_tab": 0,
        "json_ok": 0, "newlines": 0,
    }
    tx_stats = {"count": 0, "errors": 0}

    stop = threading.Event()
    buf = bytearray()
    start = time.time()
    last_report = start
    last_tx_time = start
    tx_frame = make_drv2_frame(0.0, 0.0)
    tx_period = 1.0 / rate_hz if rate_hz > 0 else 0

    # Start TX thread for layers 1-3
    if rate_hz > 0 and not single_threaded:
        t = threading.Thread(target=tx_thread_fn, args=(ser, rate_hz, stop, tx_stats), daemon=True)
        t.start()
        print(f"  TX thread started at {rate_hz} Hz")

    print(f"\n--- Listening (Ctrl-C to stop) ---\n")

    try:
        while True:
            # Single-threaded TX (layer 4)
            if single_threaded and rate_hz > 0:
                now_tx = time.time()
                if now_tx - last_tx_time >= tx_period:
                    last_tx_time = now_tx
                    try:
                        ser.write(tx_frame)
                        tx_stats["count"] += 1
                    except Exception:
                        tx_stats["errors"] += 1

            # RX
            try:
                waiting = ser.in_waiting
            except OSError:
                time.sleep(0.01)
                continue

            try:
                chunk = ser.read(waiting or 1)
            except serial.SerialException as e:
                if "returned no data" in str(e) or "readiness" in str(e):
                    time.sleep(0.01)
                    continue
                raise
            except OSError:
                time.sleep(0.01)
                continue

            if not chunk:
                continue

            if b"\n" in chunk:
                stats["newlines"] += chunk.count(b"\n")

            buf.extend(chunk)

            if len(buf) > 8192:
                buf.clear()
                continue

            while b"\x00" in buf:
                idx = buf.index(b"\x00")
                raw = bytes(buf[:idx])
                del buf[:idx + 1]
                stats["rx_frames"] += 1

                if not raw:
                    continue

                decoded = cobs_decode(raw)
                if not decoded:
                    stats["cobs_fail"] += 1
                    # Show first few failures
                    if stats["cobs_fail"] <= 3:
                        print(f"  [COBS FAIL] {len(raw)}B: {raw[:40].hex(' ')}")
                        printable = "".join(chr(b) if 32 <= b < 127 else "." for b in raw[:60])
                        print(f"    ascii: {printable}")
                    continue

                stats["cobs_ok"] += 1
                decoded_str = decoded.decode("utf-8", errors="replace")

                tab_idx = decoded_str.rfind("\t")
                if tab_idx < 0:
                    stats["crc_no_tab"] += 1
                    if stats["crc_no_tab"] <= 3:
                        print(f"  [NO TAB] {decoded_str[:80]!r}")
                    continue

                json_part = decoded_str[:tab_idx]
                crc_hex = decoded_str[tab_idx + 1:]

                try:
                    expected = int(crc_hex, 16)
                except ValueError:
                    stats["crc_fail"] += 1
                    continue

                actual = crc16(decoded[:tab_idx])
                if actual != expected:
                    stats["crc_fail"] += 1
                    continue

                stats["crc_ok"] += 1

                try:
                    obj = json.loads(json_part)
                    stats["json_ok"] += 1
                    # Print first few good messages
                    if stats["json_ok"] <= 5:
                        t = obj.get("type", "?")
                        print(f"  [OK] type={t} | {json_part[:80]}")
                except Exception:
                    pass

            # Report every 5s
            now = time.time()
            if now - last_report >= 5.0:
                last_report = now
                elapsed = now - start

                print(f"\n  --- {elapsed:.0f}s | Layer {layer} | TX {rate_hz}Hz {'(1-thread)' if single_threaded else ''} ---")
                print(f"  RX frames:  {stats['rx_frames']}")
                print(f"  COBS OK:    {stats['cobs_ok']}   FAIL: {stats['cobs_fail']}")
                print(f"  CRC OK:     {stats['crc_ok']}   FAIL: {stats['crc_fail']}   no_tab: {stats['crc_no_tab']}")
                print(f"  JSON OK:    {stats['json_ok']}")
                print(f"  TX sent:    {tx_stats['count']}   errors: {tx_stats['errors']}")
                if stats["newlines"]:
                    print(f"  NEWLINES:   {stats['newlines']} (!!)")
                total = stats["cobs_ok"] + stats["cobs_fail"]
                if total > 0:
                    print(f"  Success:    {100 * stats['cobs_ok'] / total:.1f}%")
                print()

    except KeyboardInterrupt:
        stop.set()
        elapsed = time.time() - start
        total = stats["cobs_ok"] + stats["cobs_fail"]
        pct = f"{100 * stats['cobs_ok'] / total:.1f}%" if total > 0 else "N/A"

        print(f"\n{'='*60}")
        print(f"FINAL | Layer {layer} | TX {rate_hz}Hz | {elapsed:.0f}s")
        print(f"{'='*60}")
        print(f"  RX frames:  {stats['rx_frames']}")
        print(f"  COBS OK:    {stats['cobs_ok']}   FAIL: {stats['cobs_fail']}")
        print(f"  CRC OK:     {stats['crc_ok']}   FAIL: {stats['crc_fail']}   no_tab: {stats['crc_no_tab']}")
        print(f"  JSON OK:    {stats['json_ok']}")
        print(f"  TX sent:    {tx_stats['count']}   errors: {tx_stats['errors']}")
        print(f"  Newlines:   {stats['newlines']}")
        print(f"  Success:    {pct}")
        print()


if __name__ == "__main__":
    main()
