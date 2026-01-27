# `speech/` folder

Speech + “voice” plumbing for the DARO robot stack.

This folder currently contains:
- **`piper_speak.py`** — Stream text-to-speech (TTS) audio from a local **Wyoming Piper** server and play it immediately via `aplay` (no intermediate `.wav` file).
- **`daro_llm.py`** — A lightweight wrapper around **Ollama**’s chat API that gives the robot a system prompt, “modes,” and small, Pi-friendly conversation memory.

---

## `piper_speak.py` — Piper (Wyoming) TTS streamer

### What it does
Connects to a Wyoming protocol server (defaults to `127.0.0.1:10200`), sends a `synthesize` request with your text, then:
1. Reads `audio-start` to learn the audio format (rate / width / channels).
2. Spawns `aplay` in *raw* mode with the correct parameters.
3. Streams `audio-chunk` payload bytes directly into `aplay`’s stdin.
4. Stops on `audio-stop`.

### Key pieces
- **`WyomingEvent`**: Simple container for `(type, data, payload)`.
- **`read_event(sock)`**: Implements Wyoming framing:
  - newline-terminated JSON header
  - optional extra JSON block (`data_length`)
  - optional binary payload (`payload_length`)
- **`PiperStreamer.speak(text)`**: Orchestrates the session and audio playback.
- **`_aplay_format(width_bytes)`**: Maps sample width to an `aplay -f` format (e.g., 2 → `S16_LE`).

### Requirements
- A running Wyoming Piper server on the configured host/port.
- ALSA installed with `aplay` available (usually `alsa-utils` package).
- Audio output configured correctly on the Raspberry Pi.

### Run it
```bash
python3 piper_speak.py
```

Or use it from other code:
```python
from speech.piper_speak import PiperStreamer

PiperStreamer(host="127.0.0.1", port=10200).speak("Hello from DARO.")
```

### Notes / gotchas
- If you see `Received audio-chunk before audio-start`, the server protocol sequence is unexpected or the client is out of sync.
- If `aplay` fails silently, confirm ALSA output device settings and that the Pi user has audio permissions.
- If the server is remote, network jitter can cause stuttering (still works, but audio may be less smooth).

---

## `daro_llm.py` — DARO LLM wrapper (Ollama Client)

### What it does
Provides a clean, robot-friendly interface over `ollama.Client.chat()` with:
- A default **system prompt** (“You are DARO…”).
- Switchable **modes** (copilot / mechanic / personality / telemetry / silly).
- Optional short **conversation memory** (trimmed for Pi constraints).
- Optional **JSON output** enforcement via `format="json"` or a provided JSON schema.

### Key pieces
- **`DEFAULT_SYSTEM_PROMPT`**: Safety + personality + constraints (short answers, no fake sensor claims, etc.).
- **`DEFAULT_MODES`**: Mode-specific constraints injected as an additional system message.
- **`DaroConfig`**: Runtime knobs:
  - `model`, `base_url`, `temperature`, `num_predict`
  - `keep_alive` (0 unloads after response to save RAM)
  - `remember_messages`, `max_history_messages`
  - `num_thread` (limits CPU threads used by the model)
- **`DaroLLM.ask(...)`**: Main API:
  - Builds messages in this order:
    1) system prompt  
    2) mode prompt (system-level)  
    3) recent history (optional)  
    4) user message (plus JSON instruction if requested)
  - Calls `Client.chat(...)` with configured options.
  - Optionally parses JSON responses.

### Requirements
- Ollama running and reachable at `base_url` (default: `http://127.0.0.1:11434`).
- Python package `ollama` installed in your venv:
  ```bash
  pip install ollama
  ```
- A model pulled in Ollama matching `DaroConfig.model` (default: `tinyllama`).

### Run it
```bash
python3 daro_llm.py
```

Example usage:
```python
from speech.daro_llm import DaroLLM, DaroConfig

daro = DaroLLM(DaroConfig(model="tinyllama", temperature=0.2))
print(daro.ask("Hi DARO, what's the plan?", mode="copilot"))
print(daro.ask("Diagnose why my motor won't spin.", mode="mechanic"))
```

JSON-only response:
```python
out = daro.ask("Return your intent and confidence.", json_only=True)
# out is a dict (parsed JSON)
```

Schema-constrained response:
```python
schema = {
  "type": "object",
  "properties": {
    "intent": {"type": "string"},
    "reply": {"type": "string"},
    "confidence": {"type": "number", "minimum": 0, "maximum": 1}
  },
  "required": ["intent", "reply", "confidence"]
}

out = daro.ask("Summarize what we should do next.", schema=schema)
print(out["reply"], out["confidence"])
```

### Notes / gotchas
- **System prompt injection**: Ollama’s `system=` parameter is not used here; the system prompt is added as the first `{"role":"system"}` message (more portable and explicit).
- **History is trimmed hard**: by design, to avoid ballooning RAM usage on a Raspberry Pi.
- If JSON parsing fails, `ask()` raises a `ValueError` with the raw output to help debugging.

---

## Suggested layout / imports

If your project structure is:
```
pi/app/
  speech/
    piper_speak.py
    daro_llm.py
```

Then you typically want an `__init__.py` in `speech/` so imports work cleanly:
```bash
touch speech/__init__.py
```

And you’ll import with:
```python
from speech.daro_llm import DaroLLM
from speech.piper_speak import PiperStreamer
```
