from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

from ollama import Client

JsonSchema = Dict[str, Any]


DEFAULT_SYSTEM_PROMPT = """\
You are DARO, a small robot car assistant with a playful voice.

Hard rules:
- Answer the user's LAST message directly. No transcripts, no "User:"/"DARO:" labels, no scripts unless asked.
- Stay on-topic. Don’t introduce unrelated tasks or rewrite prompts.

Style:
- Be silly if appropriate, but be coherent.
- 1–2 short sentences by default. Max 3 sentences.
- No profanity. No unsafe/illegal/harmful advice.
"""


DEFAULT_MODES: Dict[str, str] = {
    "copilot": "Be concise. Offer 1–3 options with tradeoffs. Avoid long explanations.",
    "mechanic": "Focus on diagnostics, wiring sanity checks, and minimal reproducible tests.",
    "personality": "Add light charm and car metaphors, but never sacrifice correctness.",
    "telemetry": "Summarize provided sensor values, detect anomalies, propose next actions. No speculation.",
    "silly": "Be playful and friendly. Reply directly to the user. No scripts or labels. Keep it short."
}


@dataclass
class DaroConfig:
    model: str = "tinyllama"
    base_url: str = "http://127.0.0.1:11434"
    temperature: float = 0.3
    num_predict: int = 80
    keep_alive: Union[int, float, str, None] = 0  # 0 unloads after response (RAM-friendly)
    remember_messages: bool = False
    max_history_messages: int = 12  # keep small on a Pi
    num_thread: int = 3 # one less than core count


class DaroLLM:
    """
    DARO wrapper around Ollama Client.chat().

    Key fix: `system` is NOT a parameter; provide system prompt as a message:
      {"role": "system", "content": "..."}
    """

    def __init__(
        self,
        config: Optional[DaroConfig] = None,
        *,
        system_prompt: str = DEFAULT_SYSTEM_PROMPT,
        modes: Optional[Dict[str, str]] = None,
        default_mode: str = "copilot",
    ) -> None:
        self.cfg = config or DaroConfig()
        self.client = Client(host=self.cfg.base_url)

        self._system_prompt = system_prompt.strip()
        self.modes: Dict[str, str] = dict(DEFAULT_MODES if modes is None else modes)

        if default_mode not in self.modes:
            raise KeyError(f"Unknown default_mode '{default_mode}'. Available: {sorted(self.modes.keys())}")
        self._active_mode = default_mode

        self._history: List[Dict[str, str]] = []

    # -------- config surface --------

    def set_system_prompt(self, prompt: str) -> None:
        self._system_prompt = prompt.strip()

    def add_mode(self, name: str, prompt: str) -> None:
        self.modes[name] = prompt.strip()

    def set_mode(self, name: str) -> None:
        if name not in self.modes:
            raise KeyError(f"Unknown mode '{name}'. Available: {sorted(self.modes.keys())}")
        self._active_mode = name

    def clear_history(self) -> None:
        self._history.clear()

    # -------- main API --------

    def ask(
        self,
        user_text: str,
        *,
        mode: Optional[str] = None,
        schema: Optional[JsonSchema] = None,
        json_only: bool = False,
        extra_system: Optional[str] = None,
    ) -> Union[str, Dict[str, Any]]:
        """
        Returns:
          - str if no schema/json_only
          - dict if schema provided or json_only=True (parsed JSON)

        Notes:
          - System prompt is injected as the first system message.
          - Mode is injected as an additional system message (so it acts like behavior constraints).
        """
        chosen_mode = mode or self._active_mode
        if chosen_mode not in self.modes:
            raise KeyError(f"Unknown mode '{chosen_mode}'. Available: {sorted(self.modes.keys())}")

        # Build messages in the recommended order:
        messages: List[Dict[str, str]] = []

        # 1) system
        sys = self._system_prompt
        if extra_system:
            sys = f"{sys}\n\n{extra_system.strip()}"
        messages.append({"role": "system", "content": sys})

        # 2) mode as system-level behavior constraints
        messages.append({"role": "system", "content": f"Mode: {chosen_mode}. {self.modes[chosen_mode]}"})

        # 3) history (optional)
        if self.cfg.remember_messages and self._history:
            messages.extend(self._history[-self.cfg.max_history_messages * 2 :])

        # 4) user message (with JSON instruction if requested)
        user_payload = user_text.strip()
        if schema is not None:
            user_payload += "\n\nReturn ONLY JSON that matches the provided schema."
        elif json_only:
            user_payload += "\n\nReturn ONLY valid JSON. No extra text."
        # 5 conversation guard
        messages.append({"role": "system", "content": "Answer the user directly. No roleplay transcripts. No numbered dialogue unless asked."})
        

        messages.append({"role": "user", "content": user_payload})

        options = {
            "temperature": self.cfg.temperature,
            "num_predict": self.cfg.num_predict,
            "num_thread":self.cfg.num_thread
        }

        resp = self.client.chat(
            model=self.cfg.model,
            messages=messages,
            format=(schema if schema is not None else ("json" if json_only else None)),
            options=options,
            keep_alive=self.cfg.keep_alive,
            stream=False,
        )

        content = (resp.get("message") or {}).get("content", "")
        if not isinstance(content, str):
            content = str(content)

        # Update history (store clean user + assistant)
        if self.cfg.remember_messages:
            self._history.append({"role": "user", "content": user_text.strip()})
            self._history.append({"role": "assistant", "content": content})

            # hard trim
            max_items = self.cfg.max_history_messages * 2
            if len(self._history) > max_items:
                self._history = self._history[-max_items:]

        # Parse JSON if requested
        if schema is not None or json_only:
            try:
                return json.loads(content)
            except json.JSONDecodeError as e:
                raise ValueError(f"Non-JSON output (parse failed): {e}\nRaw:\n{content}") from e

        return content

    def reply_text(self, user_text: str, *, mode: Optional[str] = None, extra_system: Optional[str] = None) -> str:
        """
        Guaranteed to return only the assistant reply text (no transcripts),
        by forcing JSON output and extracting the 'reply' field.
        """
        schema = {
            "type": "object",
            "properties": {
                "reply": {"type": "string"}
            },
            "required": ["reply"]
        }

        data = self.ask(
            user_text,
            mode=mode,
            schema=schema,         # forces structured output
            extra_system=(
                (extra_system or "").strip()
                + "\n\nReturn JSON with a single key 'reply'. 'reply' must be ONLY what DARO says (no 'User:' labels)."
            ).strip()
        )

        if not isinstance(data, dict) or "reply" not in data:
            return "Not sure—try again."
        return str(data["reply"]).strip()

if "__main__" == __name__:
    daro = DaroLLM(DaroConfig(model="tinyllama", temperature=0.2))
    print(daro.reply_text("Hi Daro, whats the weather like?", mode="silly"))

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