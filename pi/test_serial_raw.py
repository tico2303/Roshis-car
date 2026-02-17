#!/usr/bin/env python3
"""
Raw serial diagnostic — reads bytes from ESP32 and shows exactly what arrives.
Checks COBS framing, COBS decode, CRC verification at each stage.

Usage: python3 test_serial_raw.py [port] [baud]
  Default: /dev/ttyUSB0 115200
"""

import json
import sys
import serial
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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} @ {baud}")
    ser = serial.Serial(port, baudrate=baud, timeout=1)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()

    buf = bytearray()
    stats = {"frames": 0, "cobs_ok": 0, "cobs_fail": 0, "crc_ok": 0, "crc_fail": 0, "json_ok": 0, "empty": 0, "newlines": 0}
    start = time.time()
    last_stats_time = start

    print("\n--- Listening for COBS frames (0x00 delimited) ---")
    print("(also checking for newline-delimited legacy frames)\n")

    # Show first N raw chunks for debugging
    raw_chunks_shown = 0
    MAX_RAW_CHUNKS = 10

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

            # Show raw bytes for first few chunks
            if raw_chunks_shown < MAX_RAW_CHUNKS:
                raw_chunks_shown += 1
                hex_str = chunk[:64].hex(" ")
                ascii_str = chunk[:64].decode("ascii", errors="replace")
                # Mark non-printable
                ascii_str = "".join(c if 32 <= ord(c) < 127 else "." for c in ascii_str)
                print(f"  [RAW CHUNK #{raw_chunks_shown}] {len(chunk)} bytes: {hex_str}")
                print(f"  [RAW ASCII]  {ascii_str!r}")
                has_nulls = b"\x00" in chunk
                has_newlines = b"\n" in chunk
                print(f"    has 0x00: {has_nulls}  has 0x0a(\\n): {has_newlines}")
                print()

            buf.extend(chunk)

            # Count newlines (indicates old firmware still running)
            nl_count = chunk.count(b"\n")
            if nl_count:
                stats["newlines"] += nl_count

            # Process COBS frames
            while b"\x00" in buf:
                idx = buf.index(b"\x00")
                raw = bytes(buf[:idx])
                del buf[:idx + 1]
                stats["frames"] += 1

                if not raw:
                    stats["empty"] += 1
                    continue

                # Try COBS decode
                decoded = cobs_decode(raw)
                if not decoded:
                    stats["cobs_fail"] += 1
                    raw_hex = raw[:40].hex(" ")
                    if stats["cobs_fail"] <= 5:
                        print(f"  [COBS FAIL] frame #{stats['frames']}: {len(raw)} bytes: {raw_hex}")
                    continue

                stats["cobs_ok"] += 1
                decoded_str = decoded.decode("utf-8", errors="replace")

                # Check for CRC tab separator
                tab_idx = decoded_str.rfind("\t")
                if tab_idx < 0:
                    if stats["cobs_ok"] <= 5:
                        print(f"  [NO TAB] frame #{stats['frames']}: {decoded_str[:100]!r}")
                    continue

                json_part = decoded_str[:tab_idx]
                crc_hex_str = decoded_str[tab_idx + 1:]

                # Verify CRC
                try:
                    expected = int(crc_hex_str, 16)
                except ValueError:
                    stats["crc_fail"] += 1
                    if stats["crc_fail"] <= 5:
                        print(f"  [BAD CRC HEX] crc_field={crc_hex_str!r} json={json_part[:80]!r}")
                    continue

                actual = crc16(decoded[:tab_idx])
                if actual != expected:
                    stats["crc_fail"] += 1
                    if stats["crc_fail"] <= 5:
                        print(f"  [CRC MISMATCH] expected={expected:04x} actual={actual:04x}")
                        print(f"    json ({len(json_part)}b): {json_part[:120]!r}")
                    continue

                stats["crc_ok"] += 1

                # Try JSON parse
                try:
                    obj = json.loads(json_part)
                    stats["json_ok"] += 1
                    msg_type = obj.get("type", "?")
                    if stats["json_ok"] <= 20:
                        print(f"  [OK] type={msg_type} | {json_part[:100]}")
                except Exception as e:
                    if stats["crc_ok"] <= 5:
                        print(f"  [JSON FAIL] {e} | {json_part[:100]!r}")

            # Also try to find newline-delimited messages (detect old firmware)
            while b"\n" in buf:
                idx = buf.index(b"\n")
                line = bytes(buf[:idx]).strip(b" \r")
                # Don't consume — leave in buffer for COBS processing
                # Just report that we saw newline-delimited content
                if line and stats["newlines"] <= 5:
                    text = line.decode("utf-8", errors="replace")
                    print(f"  [NEWLINE MSG] {text[:120]!r}")
                    print(f"    ^^^ This means ESP32 is sending OLD newline-framed data!")
                break  # only check once per loop

            # Periodic stats every 5s
            now = time.time()
            if now - last_stats_time >= 5.0:
                last_stats_time = now
                elapsed = now - start
                print(f"\n  --- Stats ({elapsed:.0f}s) ---")
                print(f"  0x00 frames:    {stats['frames']} (empty: {stats['empty']})")
                print(f"  COBS decode OK: {stats['cobs_ok']}  FAIL: {stats['cobs_fail']}")
                print(f"  CRC OK:         {stats['crc_ok']}  FAIL: {stats['crc_fail']}")
                print(f"  JSON OK:        {stats['json_ok']}")
                print(f"  Newlines seen:  {stats['newlines']}")
                if stats["newlines"] > 0 and stats["frames"] == 0:
                    print(f"  >>> ESP32 is NOT sending COBS! Flash new firmware. <<<")
                print(f"  Buffer: {len(buf)} bytes pending")
                print()

    except KeyboardInterrupt:
        elapsed = time.time() - start
        print(f"\n\n=== Final Stats ({elapsed:.1f}s) ===")
        for k, v in stats.items():
            print(f"  {k:16s}: {v}")
        if stats["newlines"] > 0 and stats["cobs_ok"] == 0:
            print(f"\n  DIAGNOSIS: ESP32 is sending newline-framed data (old firmware).")
            print(f"  Flash the new COBS firmware: cd esp32 && pio run -e main -t upload")
        elif stats["cobs_fail"] > stats["cobs_ok"]:
            print(f"\n  DIAGNOSIS: COBS frames arriving but mostly failing decode.")
            print(f"  Likely electrical noise. Check USB cable and motor EMI.")
        elif stats["crc_fail"] > stats["crc_ok"]:
            print(f"\n  DIAGNOSIS: COBS framing OK but CRC failing.")
            print(f"  Possible encode/decode mismatch. Check COBS implementations match.")
        elif stats["json_ok"] > 0:
            print(f"\n  DIAGNOSIS: Protocol working! {stats['json_ok']} good messages received.")


if __name__ == "__main__":
    main()
