# app/audio_input/mic.py
from __future__ import annotations

import json
import queue
import re
import time
from dataclasses import dataclass
from typing import Optional
import sounddevice as sd
from vosk import Model, KaldiRecognizer


DARO_VARIANTS = [
    "darrow", "dario", "darko", "darryl", "dora", "narrow", "doral", "darling",
    "doro", "door row", "darren", "dara", "dar row", "dar round", "durrani", "goro"
]


def _now_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class MicConfig:
    model_path: str
    sample_rate: int = 16000
    blocksize: int = 4000          # smaller chunks => faster partial updates
    device: Optional[int] = None
    wake_enabled: bool = True
    wake_phrase: str = "hey"
    name_variants: tuple[str, ...] = tuple(DARO_VARIANTS)
    min_chars: int = 1

    debug: bool = True
    partial_debug: bool = True
    partial_print_interval_ms: int = 250  # throttle partial prints


class VoskMic:
    def __init__(self, cfg: MicConfig) -> None:
        self.cfg = cfg
        self._q: "queue.Queue[bytes]" = queue.Queue()
        self._model = Model(self.cfg.model_path)
        self._rec = KaldiRecognizer(self._model, self.cfg.sample_rate)

        variants = sorted(set(v.strip().lower() for v in self.cfg.name_variants if v.strip()))
        name_alt = "|".join(re.escape(v) for v in variants)
        self._wake_re = re.compile(rf"\b{re.escape(self.cfg.wake_phrase)}\s+({name_alt})\b", re.I)

        self._last_partial_print_ms = 0

    def _callback(self, indata, frames, time_info, status) -> None:
        if status and self.cfg.debug:
            print(f"[mic] status: {status}", flush=True)
        self._q.put(bytes(indata))

    def _strip_wake(self, text: str) -> Optional[str]:
        t = text.strip()
        if not t:
            return None
        if not self.cfg.wake_enabled:
            return t

        # IMPORTANT: use the original text for slicing, but match case-insensitively
        m = self._wake_re.search(t)
        if not m:
            return None

        remainder = t[m.end():].strip(" ,.!?:;-")
        # Wake detected. remainder may be "" if user only said "hey darryl".
        if len(remainder) < self.cfg.min_chars:
            return ""  # <-- this is the key fix
        return remainder


    def listen_once(self) -> Optional[str]:
        start_ms = _now_ms()
        if self.cfg.debug:
            print(f"[mic] listening... (wake={'on' if self.cfg.wake_enabled else 'off'})")

        with sd.RawInputStream(
            samplerate=self.cfg.sample_rate,
            blocksize=self.cfg.blocksize,
            dtype="int16",
            channels=1,
            callback=self._callback,
            device=self.cfg.device,
        ):
            while True:
                data = self._q.get()

                if self._rec.AcceptWaveform(data):
                    result = json.loads(self._rec.Result())
                    final_text = (result.get("text") or "").strip()
                    if self.cfg.debug and final_text:
                        dt = _now_ms() - start_ms
                        print(f"[mic] FINAL (+{dt}ms): {final_text}")

                    cleaned = self._strip_wake(final_text)

                    if self.cfg.debug and final_text:
                        if cleaned is not None:
                            print(f"[mic] WAKE DETECTED -> remainder='{cleaned}'")
                        else:
                            print("[mic] wake not detected (ignored)")

                    if cleaned is not None:
                        return cleaned

                else:
                    if not self.cfg.partial_debug:
                        continue
                    partial = (json.loads(self._rec.PartialResult()).get("partial") or "").strip()
                    if not partial:
                        continue
                    now = _now_ms()
                    if now - self._last_partial_print_ms >= self.cfg.partial_print_interval_ms:
                        self._last_partial_print_ms = now
                        dt = now - start_ms
                        print(f"[mic] ... (+{dt}ms): {partial}")

    def listen_forever(self):
        while True:
            txt = self.listen_once()
            if txt and not None:
                yield txt
