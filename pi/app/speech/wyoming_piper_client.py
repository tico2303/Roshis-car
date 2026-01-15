from __future__ import annotations

import json
import socket
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

"""
Interfaces with piper tts via the Wyoming protocol.
"""
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
    # 1) Read JSON header line (ends with \n)
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

    # 2) Optional additional data (JSON) — merged into `data`
    if data_len:
        extra = _read_exact(sock, data_len)
        extra_data = json.loads(extra.decode("utf-8"))
        data.update(extra_data)

    # 3) Optional payload (binary)
    payload = _read_exact(sock, payload_len) if payload_len else b""

    return WyomingEvent(type=event_type, data=data, payload=payload)


def send_event(sock: socket.socket, event_type: str, data: Optional[dict] = None) -> None:
    header = {"type": event_type, "data": data or {}}
    line = (json.dumps(header, separators=(",", ":")) + "\n").encode("utf-8")
    sock.sendall(line)


def synthesize_to_wav(
    text: str,
    *,
    host: str = "127.0.0.1",
    port: int = 10200,
    out_wav: Path = Path("out.wav"),
) -> Path:
    pcm_chunks = []
    fmt: Optional[Tuple[int, int, int]] = None  # (rate, width_bytes, channels)

    with socket.create_connection((host, port), timeout=10) as sock:
        send_event(sock, "synthesize", {"text": text})

        while True:
            ev = read_event(sock)

            if ev.type == "audio-start":
                fmt = (int(ev.data["rate"]), int(ev.data["width"]), int(ev.data["channels"]))
            elif ev.type == "audio-chunk":
                pcm_chunks.append(ev.payload)
            elif ev.type == "audio-stop":
                break
            # ignore other event types if present

    if not fmt:
        raise RuntimeError("Did not receive audio-start (no audio format).")

    rate, width, channels = fmt
    out_wav.parent.mkdir(parents=True, exist_ok=True)

    with wave.open(str(out_wav), "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(width)
        wf.setframerate(rate)
        wf.writeframes(b"".join(pcm_chunks))

    return out_wav


if __name__ == "__main__":
    wav_path = synthesize_to_wav("Hi my name is Daro bot")
    print(f"Wrote {wav_path}")