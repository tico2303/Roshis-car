from pathlib import Path

from app.sound import SoundBank, QueuedMpg123Player

bank = SoundBank.from_yaml(Path(__file__).resolve().parent.parent / "sounds" / "soundbank.yaml")
player = QueuedMpg123Player(mpg123_path="/usr/bin/mpg123")

# Play an error sound (non-blocking)
clip = bank.pick(["event:error"])
print("Resolved path:", clip.path.resolve(), "is_file:", clip.path.is_file())
print("Playing:", clip.id, clip.path.name)
player.play(clip.path)

# Play a sarcastic sound, but keep it kid-safe
clip2 = bank.pick(["intent:sarcasm", "rating:clean"])
print("Playing:", clip2.id, clip2.path.name)
player.play(clip2.path)

#If you want to see why a query fails without crashing, add a helper
def debug_matches(bank: SoundBank, tags: list[str]) -> None:
    matches = bank.filter(tags)
    print(f"Query {tags} matched {len(matches)} clips:")
    for c in matches:
        print(" -", c.id, c.path.name)