from app.sound.soundbank import SoundBank

def pick_any_mood(bank:SoundBank, moods: list[str]):
    for mood in moods:
        try:
            return bank.pick([f"mood:{mood}"])
        except LookupError:
            continue
    raise LookupError("No matching mood found")

#If you want to see why a query fails without crashing, add a helper
def debug_matches(bank: SoundBank, tags: list[str]) -> None:
    matches = bank.filter(tags)
    print(f"Query {tags} matched {len(matches)} clips:")
    for c in matches:
        print(" -", c.id, c.path.name)