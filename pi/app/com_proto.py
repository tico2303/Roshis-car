#from utils import *
import serial
from typing import Dict, Any, Optional
import json
from utils import _escape_bytes, now_ms
import time

# ----------------------------
# Protocol layer (NDJSON over USB serial)
# ----------------------------
class NdjsonSerialProtocol:
    def __init__(self, ser: serial.Serial, *, log_tx: bool = True, log_rx: bool = True):
        self.ser = ser
        self.log_tx = log_tx
        self.log_rx = log_rx
        self._seq = 1

    def next_seq(self) -> int:
        s = self._seq
        self._seq += 1
        return s

    def send(self, obj: Dict[str, Any]) -> None:
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        raw = line.encode("utf-8")

        if self.log_tx:
            print("TX_RAW :", _escape_bytes(raw))

        self.ser.write(raw)
        self.ser.flush()

    def recv(self, timeout_s: float = 0.05) -> Optional[Dict[str, Any]]:
        self.ser.timeout = timeout_s
        raw = self.ser.readline()
        if not raw:
            return None

        if self.log_rx:
            print("RX_RAW :", _escape_bytes(raw))

        s = raw.decode("utf-8", "backslashreplace").strip()
        try:
            return json.loads(s)
        except Exception as e:
            # Keep non-fatal; bad boot fragments happen.
            if self.log_rx:
                print("RX_ERR :", repr(e))
            return {"type": "err", "msg": "pi_json_decode_failed", "raw": s}

    def drain(self, duration_s: float = 1.0) -> None:
        start = time.time()
        while time.time() - start < duration_s:
            _ = self.recv(timeout_s=0.05)

    # Convenience message builders
    def send_hello(self, name: str = "pi-brain") -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "hello", "name": name, "proto": "ndjson", "seq": seq, "ts": now_ms()})
        return seq

    def send_ping(self, t_ms: int) -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "ping", "t": t_ms, "seq": seq, "ts": now_ms()})
        return seq

    def send_mode(self, mode_name: str) -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "mode", "name": mode_name, "seq": seq, "ts": now_ms()})
        return seq

    def send_drv(self, thr: int, steer: int = 0, *, ms: int = 200, src: str = "xbox") -> int:
        seq = self.next_seq()
        self.send({"v": 1, "type": "drv", "thr": thr, "str": steer, "ms": ms, "src": src, "seq": seq, "ts": now_ms()})
        return seq
