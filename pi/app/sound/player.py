from __future__ import annotations

import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path
from queue import Queue
from typing import Optional, Protocol, Tuple


class SoundPlayer(Protocol):
    def play(self, path: Path, *, block: bool = True) -> None: ...


@dataclass(frozen=True)
class Mpg123Player:
    """Simple MP3 playback via system mpg123."""
    mpg123_path: str = "mpg123"

    def play(self, path: Path, *, block: bool = True) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"Not a file: {path}")

        cmd = [self.mpg123_path, str(path)]
        if block:
            p = subprocess.run(cmd, capture_output=True, text=True)
            if p.returncode != 0:
                print("[mpg123] rc:", p.returncode)
                print("[mpg123] stdout:", p.stdout)
                print("[mpg123] stderr:", p.stderr)
        else:
            subprocess.Popen(cmd)

@dataclass
class QueuedMpg123Player:
    """
    Serializes playback so sounds can play back-to-back.

    Semantics:
      - block=False: enqueue and return immediately
      - block=True : enqueue and wait until THIS clip finishes playing
    """
    mpg123_path: str = "mpg123"
    _q: "Queue[Tuple[Optional[Path], Optional[threading.Event]]]" = field(default_factory=Queue, init=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False)
    _thread: threading.Thread = field(init=False)
    _backend: Mpg123Player = field(init=False)

    def __post_init__(self) -> None:
        self._backend = Mpg123Player(mpg123_path=self.mpg123_path)
        self._thread = threading.Thread(target=self._worker, name="sound-queue", daemon=True)
        self._thread.start()

    def play(self, path: Path, *, block: bool = True) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"Not a file: {path}")

        done = threading.Event() if block else None
        self._q.put((path, done))
        if done is not None:
            done.wait()

    def close(self) -> None:
        """Stop worker thread (optional)."""
        self._stop.set()
        self._q.put((None, None))  # sentinel
        self._thread.join(timeout=2)

    def _worker(self) -> None:
        while not self._stop.is_set():
            path, done = self._q.get()

            if path is None:  # sentinel
                if done is not None:
                    done.set()
                return

            try:
                # Block here to keep ordering deterministic
                self._backend.play(path, block=True)
            except Exception as e:
                # IMPORTANT: don't silently swallow while you're debugging
                print(f"[sound] playback error for {path}: {e}")
            finally:
                if done is not None:
                    done.set()