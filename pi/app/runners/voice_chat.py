# app/runners/voice_chat.py
from __future__ import annotations

import time

from app.audio_input.mic import VoskMic, MicConfig
from app.speech.daro_llm import DaroLLM, DaroConfig
from app.speech.piper_speak import PiperStreamer
from app.speech.wake import WakeSession


def ms() -> int:
    return int(time.time() * 1000)


def main() -> None:
    mic = VoskMic(MicConfig(
        model_path="/home/pi/Code/Roshis-car/pi/app/models/vosk-model-small-en-us-0.15",
        wake_enabled=True,
        debug=True,
        partial_debug=True,
        blocksize=4000,
    ))

    llm = DaroLLM(DaroConfig(
        temperature=0.2,
        num_predict=80,     # BIG responsiveness win
        keep_alive="5m",    # keep model loaded between turns
        num_thread=3,
    ))
    llm.clear_history() # prevents previous conversation context from poluting current

    wake = WakeSession(mic,armed_timeout_s=6.0)
    tts = PiperStreamer()

    print("[runner] Voice loop ready. Say: 'hey daro ...'")

    while True:
        t0 = ms()
        user_text = wake.next_user_utterance()
        t1 = ms()
        if user_text is None:
            continue

        # Wake-only: don't call LLM
        if user_text == "":
            tts.speak("Yeah? What's the mission?")
            continue

        print(f"[runner] MIC → DARO: '{user_text}'")

        # Force short answers (see section below)
        extra_system = "Keep the response under 12 words unless safety requires more. No lists."
        if "weather" in user_text.lower():
            extra_system = "If asked about weather, ask for city/state and you need internet connectivity first."
        t2 = ms()

        reply = llm.reply_text(user_text, mode="silly", extra_system=extra_system)
        t3 = ms()

        print(f"[runner] DARO: {reply}  (llm={t3 - t2}ms)")

        t4 = ms()
        tts.speak(reply)
        t5 = ms()

        print(f"[runner] tts={t5 - t4}ms total={t5 - t0}ms\n")


if __name__ == "__main__":
    main()

