#!/usr/bin/env python3
from __future__ import annotations

import json
import socket
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class WyomingEvent:
    type: str
    data: dict
    payload: bytes


def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf


def read_event(sock: socket.socket) -> WyomingEvent:
    # Read JSON header line (ends with \n)
    header_bytes = b""
    while not header_bytes.endswith(b"\n"):
        b1 = sock.recv(1)
        if not b1:
            raise ConnectionError("Socket closed while reading header")
        header_bytes += b1

    header = json.loads(header_bytes.decode("utf-8"))
    event_type = header["type"]
    data = header.get("data") or {}

    data_len = int(header.get("data_length") or 0)
    payload_len = int(header.get("payload_length") or 0)

    # Optional additional data (JSON)
    if data_len:
        extra = _read_exact(sock, data_len)
        extra_data = json.loads(extra.decode("utf-8"))
        if isinstance(extra_data, dict):
            data.update(extra_data)

    payload = _read_exact(sock, payload_len) if payload_len else b""
    return WyomingEvent(type=event_type, data=data, payload=payload)


def send_event(sock: socket.socket, event_type: str, data: Optional[dict] = None) -> None:
    header = {"type": event_type, "data": data or {}}
    line = (json.dumps(header, separators=(",", ":")) + "\n").encode("utf-8")
    sock.sendall(line)


def _aplay_format(width_bytes: int) -> str:
    # aplay -f formats
    return {1: "U8", 2: "S16_LE", 3: "S24_3LE", 4: "S32_LE"}.get(width_bytes, "S16_LE")


class PiperStreamer:
    def __init__(self, host: str = "127.0.0.1", port: int = 10200):
        self.host = host
        self.port = port

    def speak(self, text: str, timeout: float = 10.0) -> None:
        text = (text or "").strip()
        if not text:
            return

        fmt: Optional[Tuple[int, int, int]] = None  # (rate, width_bytes, channels)
        aplay_proc: Optional[subprocess.Popen] = None

        with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
            send_event(sock, "synthesize", {"text": text})

            try:
                while True:
                    ev = read_event(sock)

                    if ev.type == "audio-start":
                        rate = int(ev.data["rate"])
                        width = int(ev.data["width"])
                        channels = int(ev.data["channels"])
                        fmt = (rate, width, channels)

                        aplay_proc = subprocess.Popen(
                            [
                                "aplay",
                                "-q",
                                "-t",
                                "raw",
                                "-f",
                                _aplay_format(width),
                                "-r",
                                str(rate),
                                "-c",
                                str(channels),
                            ],
                            stdin=subprocess.PIPE,
                        )

                    elif ev.type == "audio-chunk":
                        if aplay_proc is None or aplay_proc.stdin is None:
                            raise RuntimeError("Received audio-chunk before audio-start")
                        if ev.payload:
                            aplay_proc.stdin.write(ev.payload)

                    elif ev.type == "audio-stop":
                        break

                    # ignore others

            finally:
                if aplay_proc and aplay_proc.stdin:
                    try:
                        aplay_proc.stdin.close()
                    except Exception:
                        pass
                if aplay_proc:
                    aplay_proc.wait(timeout=10)

        if not fmt:
            raise RuntimeError("Did not receive audio-start (no audio format).")


if __name__ == "__main__":
    PiperStreamer().speak("Hi my name is Daro bot. Whats up Netties")