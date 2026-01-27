import json
import time
import serial
from xbox_controller import XboxController
from speech import TTSSpeaker

PORT = "/dev/ttyUSB0"  # might be /dev/ttyACM0; run: ls /dev/tty*
BAUD = 115200

"""
v: version int
type: message type string
seq: sequence number int
ts: timestamp in miliseconds int
"""
def _escape_bytes(b: bytes) -> str:
    # Makes control chars visible: \r \n etc.
    return b.decode("utf-8", "backslashreplace").replace("\n", "\\n").replace("\r", "\\r")

def send(ser: serial.Serial, obj: dict):
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    raw = line.encode("utf-8")

    #print("TX_JSON:", obj)
    print("TX_RAW :", _escape_bytes(raw))

    ser.write(raw)
    ser.flush()

def read_line(ser: serial.Serial, timeout_s: float = 1.0):
    ser.timeout = timeout_s
    raw = ser.readline()
    if not raw:
        return None

    print("RX_RAW :", _escape_bytes(raw))

    try:
        msg = json.loads(raw.decode("utf-8").strip())
        #print("RX_JSON:", msg)
        return msg
    except Exception as e:
        print("RX_ERR :", repr(e))
        return {"type": "err", "msg": "pi_json_decode_failed", "raw": _escape_bytes(raw)}


def main():
    with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
        time.sleep(1.0)  # give ESP32 time to boot/reset

        seq = 1

        # Drain any boot messages
        start = time.time()
        while time.time() - start < 1.0:
            msg = read_line(ser, timeout_s=0.1)
            if msg:
                print("RX:", msg)

        # HELLO handshake
        send(ser, {"v": 1, "type": "hello", "name": "pi-brain", "proto": "ndjson", "seq": seq})
        seq += 1

        msg = read_line(ser, timeout_s=2.0)
        print("RX:", msg)

        # Ping/pong latency test
        t_ms = int(time.time() * 1000)
        send(ser, {"v": 1, "type": "ping", "t": t_ms, "seq": seq})
        sent_seq = seq
        seq += 1

        pong = read_line(ser, timeout_s=2.0)
        print("RX:", pong)
        if pong and pong.get("type") == "pong" and pong.get("t") == t_ms:
            rtt = int(time.time() * 1000) - t_ms
            print(f"Ping OK (seq={sent_seq}) RTT ≈ {rtt} ms")

        # Drive command test (fake values)
        send(ser, {"v": 1, "type": "drv", "thr": 20, "str": -10, "seq": seq})
        seq += 1
        print("RX:", read_line(ser, timeout_s=1.0))

        send(ser, {"v": 1, "type": "mode", "name": "drama", "seq": seq})
        seq += 1
        print("RX:", read_line(ser, timeout_s=1.0))

        # Stream drive commands for 3 seconds
        print("Streaming drv for 3 seconds...")
        end = time.time() + 3.0
        thr, st = 0, 0
        while time.time() < end:
            thr = (thr + 5) % 101
            st = -st if st != 0 else 20
            send(ser, {"v": 1, "type": "drv", "thr": thr, "str": st, "seq": seq})
            seq += 1
            # non-blocking read
            msg = read_line(ser, timeout_s=0.05)
            if msg:
                print("RX:", msg)
            time.sleep(0.05)

        print("Done.")

if __name__ == "__main__":
    main()
