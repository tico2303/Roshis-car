from __future__ import annotations
import json
from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class PrintOnlyProtocol:
    seq: int = 1

    def _next(self) -> int:
        s = self.seq
        self.seq += 1
        return s

    def send_drv(self, thr: int, steer: int = 0, *, ms: int = 200, src: str = "xbox") -> int:
        obj = {"v": 1, "type": "drv", "thr": thr, "str": steer, "ms": ms, "src": src, "seq": self._next()}
        print(json.dumps(obj))
        return obj["seq"]

    def send_ping(self, t_ms: int) -> int:
        obj = {"v": 1, "type": "ping", "t": t_ms, "seq": self._next()}
        print(json.dumps(obj))
        return obj["seq"]