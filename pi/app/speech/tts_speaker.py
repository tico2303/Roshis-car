from __future__ import annotations

import hashlib
import os
import platform
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Import this from your existing file
# from wyoming_piper_client import synthesize_to_wav
from speech.wyoming_piper_client import synthesize_to_wav  # adjust import path if needed


def _default_player_cmd() -> list[str]:
    sys = platform.system().lower()
    if "darwin" in sys:
        return ["afplay"]
    # Linux / Raspberry Pi
    return ["aplay"]


def _text_hash(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


@dataclass
class TTSSpeaker:
    host: str = "127.0.0.1"
    port: int = 10200
    out_dir: Path = Path("./tts_out")
    player_cmd: Optional[list[str]] = None
    cache: bool = True

    def __post_init__(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        if self.player_cmd is None:
            self.player_cmd = _default_player_cmd()

    def say(self, text: str, *, play: bool = True, filename: Optional[str] = None) -> Path:
        """
        Synthesize `text` via Wyoming Piper and optionally play it.
        Returns the path to the generated wav.
        """
        text = (text or "").strip()
        if not text:
            raise ValueError("Text is empty.")

        if filename is None:
            # cache-friendly deterministic name
            filename = f"piper_{_text_hash(text)}.wav"
        wav_path = self.out_dir / filename

        if (not self.cache) or (not wav_path.exists()):
            synthesize_to_wav(text, host=self.host, port=self.port, out_wav=wav_path)

        if play:
            subprocess.run([*self.player_cmd, str(wav_path)], check=True)

        return wav_path