#!/usr/bin/env python3
"""
Raw serial diagnostic — reads bytes from ESP32 and shows exactly what arrives.
Checks COBS framing, COBS decode, CRC verification at each stage.

Usage: python3 test_serial_raw.py [port] [baud]
  Default: /dev/ttyUSB0 115200
"""

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
    stats = {"frames": 0, "cobs_ok": 0, "cobs_fail": 0, "crc_ok": 0, "crc_fail": 0, "json_ok": 0}
    start = time.time()

    print("\n--- Listening for COBS frames (0x00 delimited) ---")
    print("(also checking for newline-delimited legacy frames)\n")

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue

            buf.extend(chunk)

            # Check if we see newlines (legacy framing leaking through)
            if b"\n" in buf:
                nl_count = buf.count(b"\n")
                # Show what's between newlines
                parts = buf.split(b"\n")
                for p in parts[:-1]:
                    text = p.decode("utf-8", errors="replace").strip()
                    if text:
                        print(f"  [NEWLINE-DELIMITED] {text!r}")
                # Don't consume — let COBS logic handle the full buffer

            # Process COBS frames
            while b"\x00" in buf:
                idx = buf.index(b"\x00")
                raw = bytes(buf[:idx])
                del buf[:idx + 1]
                stats["frames"] += 1

                if not raw:
                    continue

                # Show raw COBS bytes
                raw_hex = raw[:32].hex(" ")
                if len(raw) > 32:
                    raw_hex += f" ... ({len(raw)} bytes total)"

                # Try COBS decode
                decoded = cobs_decode(raw)
                if not decoded:
                    stats["cobs_fail"] += 1
                    print(f"  [COBS FAIL] frame #{stats['frames']}: raw={raw_hex}")
                    continue

                stats["cobs_ok"] += 1
                decoded_str = decoded.decode("utf-8", errors="replace")

                # Check for CRC tab separator
                tab_idx = decoded_str.rfind("\t")
                if tab_idx < 0:
                    print(f"  [NO TAB] frame #{stats['frames']}: {decoded_str!r}")
                    continue

                json_part = decoded_str[:tab_idx]
                crc_hex = decoded_str[tab_idx + 1:]

                # Verify CRC
                try:
                    expected = int(crc_hex, 16)
                except ValueError:
                    stats["crc_fail"] += 1
                    print(f"  [BAD CRC HEX] frame #{stats['frames']}: crc_field={crc_hex!r} json={json_part[:80]!r}")
                    continue

                actual = crc16(decoded[:tab_idx])
                if actual != expected:
                    stats["crc_fail"] += 1
                    print(f"  [CRC MISMATCH] frame #{stats['frames']}: expected={expected:04x} actual={actual:04x}")
                    print(f"    json_part ({len(json_part)} bytes): {json_part[:120]!r}")
                    print(f"    decoded hex: {decoded[:40].hex(' ')}")
                    continue

                stats["crc_ok"] += 1

                # Try JSON parse
                import json
                try:
                    obj = json.loads(json_part)
                    stats["json_ok"] += 1
                    msg_type = obj.get("type", "?")
                    print(f"  [OK] type={msg_type} | {json_part[:100]}")
                except Exception as e:
                    print(f"  [JSON FAIL] {e} | {json_part[:100]!r}")

            # Periodic stats
            elapsed = time.time() - start
            if elapsed > 5 and stats["frames"] > 0 and int(elapsed) % 5 == 0:
                print(f"\n  --- Stats ({elapsed:.0f}s) ---")
                print(f"  frames={stats['frames']} cobs_ok={stats['cobs_ok']} cobs_fail={stats['cobs_fail']}")
                print(f"  crc_ok={stats['crc_ok']} crc_fail={stats['crc_fail']} json_ok={stats['json_ok']}")
                print()

    except KeyboardInterrupt:
        elapsed = time.time() - start
        print(f"\n\n=== Final Stats ({elapsed:.1f}s) ===")
        print(f"  Total frames:  {stats['frames']}")
        print(f"  COBS decode OK:   {stats['cobs_ok']}")
        print(f"  COBS decode FAIL: {stats['cobs_fail']}")
        print(f"  CRC OK:           {stats['crc_ok']}")
        print(f"  CRC FAIL:         {stats['crc_fail']}")
        print(f"  JSON OK:          {stats['json_ok']}")


if __name__ == "__main__":
    main()
