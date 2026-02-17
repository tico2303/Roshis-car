#!/usr/bin/env python3
"""
Offline COBS round-trip test. No serial needed.
Tests encode/decode match between Python and expected C output.
"""


def crc16(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(data: bytes) -> bytes:
    """Decode COBS (without trailing 0x00 delimiter)."""
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
    """COBS-encode, returns encoded bytes including trailing 0x00."""
    out = bytearray()
    code_idx = len(out)
    out.append(0)  # placeholder
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


def cobs_encode_reference(data: bytes) -> bytes:
    """Reference COBS encoder from Wikipedia for comparison."""
    out = bytearray()
    idx = 0
    while idx <= len(data):
        # Find next zero (or end of data)
        try:
            end = data.index(0, idx)
        except ValueError:
            end = len(data)
        segment = data[idx:end]
        # Split into chunks of 254
        while len(segment) > 0:
            chunk = segment[:254]
            segment = segment[254:]
            if len(segment) > 0 or (end < len(data) and len(chunk) < 254):
                out.append(len(chunk) + 1)
                out.extend(chunk)
            elif end >= len(data):
                # Last segment, no trailing zero
                out.append(len(chunk) + 1)
                out.extend(chunk)
            else:
                out.append(len(chunk) + 1)
                out.extend(chunk)
        idx = end + 1
        if idx > len(data):
            break
    out.append(0x00)
    return bytes(out)


# ---- Tests ----

def test_basic_roundtrip():
    """Test that encode->decode is identity."""
    test_cases = [
        b"hello",
        b'{"type":"drv2","left":0.0,"right":0.0}',
        b'{"type":"drv2","left":0.0,"right":0.0}\t1a2b',
        b'\x00',
        b'\x00\x00',
        b'abc\x00def',
        b'',
    ]
    for i, original in enumerate(test_cases):
        encoded = cobs_encode(original)
        # Strip trailing 0x00 for decode
        decoded = cobs_decode(encoded[:-1])
        if decoded == original:
            print(f"  [PASS] test {i}: {original[:40]!r}")
        else:
            print(f"  [FAIL] test {i}:")
            print(f"    original: {original!r}")
            print(f"    encoded:  {encoded.hex(' ')}")
            print(f"    decoded:  {decoded!r}")
            # Show where they differ
            for j in range(max(len(original), len(decoded))):
                o = original[j] if j < len(original) else None
                d = decoded[j] if j < len(decoded) else None
                if o != d:
                    print(f"    first diff at byte {j}: original={o} decoded={d}")
                    break


def test_full_protocol_roundtrip():
    """Simulate full ESP32 sendJson -> Pi receive path."""
    json_str = '{"v":1,"type":"sens","seq":1234,"t_ms":56789,"data":{"sensor":"imu","linear_acceleration":{"x":-9.6175,"y":0.4663,"z":2.1997},"angular_velocity":{"x":-0.0023,"y":0.0014,"z":0.0089}}}'

    json_bytes = json_str.encode("utf-8")
    crc = crc16(json_bytes)
    payload = f"{json_str}\t{crc:04x}".encode("utf-8")

    print(f"\n  Payload ({len(payload)} bytes): {payload[:80]}...")
    print(f"  CRC: {crc:04x}")

    # Check for null bytes in payload
    null_count = payload.count(b'\x00')
    print(f"  Null bytes in payload: {null_count}")

    # Encode
    encoded = cobs_encode(payload)
    print(f"  Encoded ({len(encoded)} bytes): {encoded[:40].hex(' ')}...")
    print(f"  Encoded ends with 0x00: {encoded[-1] == 0}")

    # Check no 0x00 in encoded data (except trailing)
    inner = encoded[:-1]
    inner_nulls = inner.count(b'\x00')
    print(f"  Null bytes in encoded (excl delimiter): {inner_nulls}")
    if inner_nulls > 0:
        positions = [i for i, b in enumerate(inner) if b == 0]
        print(f"    NULL POSITIONS: {positions}")

    # Decode
    decoded = cobs_decode(inner)
    print(f"  Decoded ({len(decoded)} bytes): {decoded[:80]}...")

    if decoded == payload:
        print(f"  [PASS] Round-trip matches!")
    else:
        print(f"  [FAIL] Round-trip mismatch!")
        print(f"    payload len={len(payload)} decoded len={len(decoded)}")
        for j in range(max(len(payload), len(decoded))):
            o = payload[j] if j < len(payload) else None
            d = decoded[j] if j < len(decoded) else None
            if o != d:
                print(f"    first diff at byte {j}: payload=0x{o:02x} decoded=0x{d:02x}" if o is not None and d is not None else f"    first diff at byte {j}: payload={o} decoded={d}")
                break

    # Now verify CRC on decoded data
    tab_idx = decoded.rfind(b'\t')
    if tab_idx < 0:
        print(f"  [FAIL] No tab in decoded data!")
        return

    json_part = decoded[:tab_idx]
    crc_part = decoded[tab_idx + 1:]
    print(f"  CRC field: {crc_part}")

    try:
        expected_crc = int(crc_part, 16)
    except ValueError:
        print(f"  [FAIL] Can't parse CRC hex: {crc_part!r}")
        return

    actual_crc = crc16(json_part)
    if actual_crc == expected_crc:
        print(f"  [PASS] CRC verified: {actual_crc:04x}")
    else:
        print(f"  [FAIL] CRC mismatch: expected={expected_crc:04x} actual={actual_crc:04x}")


def test_drv2_tx():
    """Simulate Pi -> ESP32 drv2 command encoding."""
    json_str = '{"type":"drv2","left":0.0,"right":0.0}'
    json_bytes = json_str.encode("utf-8")
    crc = crc16(json_bytes)
    payload = f"{json_str}\t{crc:04x}".encode("utf-8")

    print(f"\n  drv2 payload ({len(payload)} bytes): {payload}")

    encoded = cobs_encode(payload)
    print(f"  Encoded ({len(encoded)} bytes): {encoded.hex(' ')}")

    # Check for nulls in encoded (excl delimiter)
    inner = encoded[:-1]
    inner_nulls = inner.count(b'\x00')
    if inner_nulls > 0:
        print(f"  [BUG!] Null bytes in encoded inner data: {inner_nulls}")
        positions = [i for i, b in enumerate(inner) if b == 0]
        print(f"    positions: {positions}")

    decoded = cobs_decode(inner)
    if decoded == payload:
        print(f"  [PASS] Round-trip OK")
    else:
        print(f"  [FAIL] Round-trip mismatch!")
        print(f"    decoded: {decoded!r}")


if __name__ == "__main__":
    print("=== Basic round-trip tests ===")
    test_basic_roundtrip()
    print("\n=== Full protocol round-trip (IMU message) ===")
    test_full_protocol_roundtrip()
    print("\n=== drv2 TX encoding ===")
    test_drv2_tx()
