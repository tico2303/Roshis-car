import time
from dataclasses import dataclass

@dataclass
class WakeSession:
    """
    Handles wake flow for voice interaction.

    Behavior:
      - First, listen with wake required.
      - If wake-only ("") is detected, we enter an 'armed' state.
      - While armed (for a short window), we listen WITHOUT requiring wake.
      - After we capture one non-empty utterance, we return to wake-required.
    """
    mic: any  # VoskMic
    armed_timeout_s: float = 6.0

    _armed_until_ms: int = 0

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _set_wake_enabled(self, enabled: bool) -> None:
        # Safe toggle (restore later)
        self.mic.cfg.wake_enabled = enabled

    def next_user_utterance(self) -> str | None:
        """
        Returns:
          - None: keep listening
          - "": wake-only detected (caller can play prompt). Also arms session.
          - "text": actual user utterance (wake removed if wake was on)
        """
        now = self._now_ms()
        armed = now < self._armed_until_ms

        # If armed, temporarily disable wake requirement for ONE utterance
        if armed:
            self._set_wake_enabled(False)
        else:
            self._set_wake_enabled(True)

        text = self.mic.listen_once()

        # Restore wake requirement by default after we get something
        # (we'll re-arm explicitly when wake-only happens)
        self._set_wake_enabled(True)

        if text is None:
            return None

        # Wake-only => arm session and return ""
        if text == "":
            self._armed_until_ms = self._now_ms() + int(self.armed_timeout_s * 1000)
            return ""

        # If we got real text while armed, consume it and disarm.
        if armed:
            self._armed_until_ms = 0

        return text
