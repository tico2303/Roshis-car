#!/usr/bin/env python3
"""
Bare-bones serial RX test. No COBS, no CRC — just newline-delimited JSON.
Pairs with esp32/src/test/io/baseline_tx_test.cpp.

Shows:
  - Raw bytes as they arrive (first N chunks)
  - Per-message-type stats (ok / corrupt / missing seq gaps)
  - Byte rate and message rate
  - Latency (ESP32 timestamp vs wall clock drift)

Usage: python3 test_baseline_rx.py [port] [baud]
  Default: /dev/ttyUSB0 115200
"""

import json
import sys
import serial
import time


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} @ {baud}")
    print(f"Expecting plain newline-delimited JSON (no COBS, no CRC)")
    print(f"Flash ESP32 with: pio run -e test_baseline_tx -t upload\n")

    ser = serial.Serial(port, baudrate=baud, timeout=1)
    ser.reset_input_buffer()
    time.sleep(0.3)
    ser.reset_input_buffer()

    buf = bytearray()

    # Stats
    total_bytes = 0
    total_lines = 0
    good_json = 0
    bad_json = 0
    by_type = {}  # type -> {count, last_seq, gaps, last_t_ms}

    start = time.time()
    last_report = start
    raw_shown = 0
    MAX_RAW = 5

    REPORT_INTERVAL = 5.0

    print("--- Listening ---\n")

    try:
        while True:
            try:
                waiting = ser.in_waiting
            except OSError:
                time.sleep(0.1)
                continue

            try:
                chunk = ser.read(waiting or 1)
            except serial.SerialException as e:
                if "returned no data" in str(e) or "readiness" in str(e):
                    time.sleep(0.01)
                    continue
                raise
            except OSError:
                time.sleep(0.1)
                continue

            if not chunk:
                continue

            total_bytes += len(chunk)

            # Show first few raw chunks
            if raw_shown < MAX_RAW:
                raw_shown += 1
                hex_part = chunk[:60].hex(" ")
                printable = "".join(
                    chr(b) if 32 <= b < 127 else "." for b in chunk[:60]
                )
                print(f"  [RAW #{raw_shown}] {len(chunk)}B: {hex_part}")
                print(f"           ascii: {printable}")
                nulls = chunk.count(0)
                nls = chunk.count(ord("\n"))
                print(f"           nulls={nulls} newlines={nls}\n")

            buf.extend(chunk)

            # Safety cap
            if len(buf) > 8192:
                print(f"  [WARN] buffer overflow {len(buf)}B — flushing")
                buf.clear()
                continue

            # Extract newline-delimited messages
            while b"\n" in buf:
                idx = buf.index(b"\n")
                raw_line = bytes(buf[:idx])
                del buf[:idx + 1]
                total_lines += 1

                line = raw_line.strip(b"\r ")
                if not line:
                    continue

                text = line.decode("utf-8", errors="replace")

                # Try JSON parse
                try:
                    obj = json.loads(text)
                    good_json += 1
                except Exception:
                    bad_json += 1
                    if bad_json <= 10:
                        # Show the corrupt line
                        preview = text[:120]
                        print(f"  [BAD JSON #{bad_json}] ({len(text)}B): {preview!r}")
                        # Show hex of first 40 bytes
                        print(f"           hex: {line[:40].hex(' ')}")
                    continue

                msg_type = obj.get("type", "?")
                seq = obj.get("seq")
                t_ms = obj.get("t")

                if msg_type not in by_type:
                    by_type[msg_type] = {
                        "count": 0,
                        "last_seq": None,
                        "gaps": 0,
                        "gap_detail": [],
                    }

                info = by_type[msg_type]
                info["count"] += 1

                # Check sequence gaps
                if seq is not None and info["last_seq"] is not None:
                    expected = info["last_seq"] + 1
                    if seq != expected:
                        info["gaps"] += 1
                        missed = seq - expected
                        if len(info["gap_detail"]) < 5:
                            info["gap_detail"].append(
                                f"seq {info['last_seq']}->{seq} (missed {missed})"
                            )

                if seq is not None:
                    info["last_seq"] = seq

                # Print first few of each type
                if info["count"] <= 3:
                    print(f"  [OK] type={msg_type} seq={seq} len={len(text)}B: {text[:100]}")

            # Periodic report
            now = time.time()
            if now - last_report >= REPORT_INTERVAL:
                elapsed = now - start
                last_report = now
                byte_rate = total_bytes / elapsed
                line_rate = total_lines / elapsed

                print(f"\n{'='*60}")
                print(f"  Stats @ {elapsed:.0f}s")
                print(f"  Bytes: {total_bytes} ({byte_rate:.0f} B/s)")
                print(f"  Lines: {total_lines} ({line_rate:.1f}/s)")
                print(f"  JSON OK: {good_json}  BAD: {bad_json}", end="")
                if good_json + bad_json > 0:
                    pct = 100 * bad_json / (good_json + bad_json)
                    print(f"  ({pct:.1f}% corrupt)")
                else:
                    print()
                print()

                for t, info in sorted(by_type.items()):
                    rate = info["count"] / elapsed
                    print(f"  type={t:8s}  count={info['count']:5d}  ({rate:.1f}/s)  gaps={info['gaps']}")
                    for g in info["gap_detail"]:
                        print(f"    gap: {g}")

                print(f"\n  Buffer pending: {len(buf)}B")
                print(f"{'='*60}\n")

    except KeyboardInterrupt:
        elapsed = time.time() - start
        print(f"\n\n{'='*60}")
        print(f"FINAL REPORT ({elapsed:.1f}s)")
        print(f"{'='*60}")
        print(f"  Total bytes:  {total_bytes} ({total_bytes/elapsed:.0f} B/s)")
        print(f"  Total lines:  {total_lines}")
        print(f"  JSON OK:      {good_json}")
        print(f"  JSON BAD:     {bad_json}")
        if good_json + bad_json > 0:
            print(f"  Corrupt rate: {100*bad_json/(good_json+bad_json):.1f}%")
        print()
        for t, info in sorted(by_type.items()):
            print(f"  {t}: {info['count']} msgs, {info['gaps']} seq gaps")
            for g in info["gap_detail"]:
                print(f"    {g}")
        print()


if __name__ == "__main__":
    main()
