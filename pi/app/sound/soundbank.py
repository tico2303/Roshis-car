from __future__ import annotations

import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence
import yaml  


@dataclass(frozen=True)
class SoundClip:
    id: str
    path: Path
    tags: frozenset[str]
    weight: float = 1.0


class SoundBank:
    def __init__(self, clips: Sequence[SoundClip]) -> None:
        self._clips = list(clips)

    @staticmethod
    def from_yaml(manifest_path: Path) -> "SoundBank":
        data = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        base_dir = manifest_path.parent / data.get("base_dir", "clips")

        clips: list[SoundClip] = []
        for item in data["sounds"]:
            clip_path = base_dir / item["file"]
            tags = frozenset(item.get("tags", []))
            weight = float(item.get("weight", 1.0))
            clips.append(SoundClip(id=item["id"], path=clip_path, tags=tags, weight=weight))

        return SoundBank(clips)

    def filter(self, required_tags: Iterable[str]) -> list[SoundClip]:
        req = set(required_tags)
        return [c for c in self._clips if req.issubset(c.tags)]

    def pick(self, required_tags: Iterable[str], *, rng: random.Random | None = None) -> SoundClip:
        rng = rng or random
        matches = self.filter(required_tags)
        if not matches:
            raise LookupError(f"No sounds match tags: {sorted(required_tags)}")

        weights = [max(0.0, c.weight) for c in matches]
        # If all weights are 0, fall back to uniform random
        if sum(weights) <= 0:
            return rng.choice(matches)

        return rng.choices(matches, weights=weights, k=1)[0]
    
    def pick_with_fallback(self, required_tags: Iterable[str],fallbacks: list[list[str]]) -> SoundClip:
        """
        Try required_tags first, then each fallback tag set in order.
        Example:
          pick_with_fallback(["intent:sarcasm","rating:clean"], fallbacks=[
              ["intent:sarcasm"],          # allow rude if needed
              ["mood:curious"],            # generic reaction
              ["event:ack"],               # last-resort beep
          ])
        """
        try_sets = [list(required_tags)] + fallbacks
        last_err: Exception | None = None

        for tag_set in try_sets:
            try:
                return self.pick(tag_set)
            except LookupError as e:
                last_err = e

        # If everything fails, bubble up the last useful error
        raise last_err or LookupError("No sounds available")