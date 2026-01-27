Soundbank Schema README

This document explains the soundbank schema used to organize and select robot sound effects
based on meaning (emotion, intent, event) rather than filenames.

Audio files are dumb assets. All intelligence lives in metadata.

--------------------------------------------------
DIRECTORY STRUCTURE
--------------------------------------------------

sounds/
  clips/                Raw audio files (.mp3, .wav, etc.)
  soundbank.yaml        Metadata and classification

--------------------------------------------------
TOP-LEVEL SCHEMA
--------------------------------------------------

version:
  Schema version for future compatibility.

base_dir:
  Directory (relative to the YAML file) containing all audio clips.

sounds:
  List of sound entries.

--------------------------------------------------
SOUND ENTRY SCHEMA
--------------------------------------------------

Each sound entry describes one playable clip.

Fields:
```yaml
id:
  Stable, human-readable identifier.
  Used for logging and debugging.
  Convention: category.subcategory (not enforced).

file:
  Filename relative to base_dir.
  The actual audio asset.

tags:
  List of key:value tags describing when and why the sound is used.
  Tags are composable and extensible.

weight:
  Relative probability when multiple sounds match.
  Higher = more likely.
  Defaults to 1.0.
```
--------------------------------------------------
TAG CATEGORIES (CONVENTIONS)
--------------------------------------------------
```yaml
event:
  What happened in the system.
  Examples:
    event:success
    event:fail
    event:error
    event:alert
    event:startup
    event:ack

mood:
  Emotional state of the robot.
  Examples:
    mood:happy
    mood:sad
    mood:curious
    mood:annoyed
    mood:proud
    mood:mischievous
    mood:teasing

intent:
  What the robot is trying to communicate.
  Examples:
    intent:celebration
    intent:mockery
    intent:sarcasm
    intent:affection
    intent:attention
    intent:thinking
    intent:surprise

character:
  Who or what the sound resembles.
  Examples:
    character:robotic
    character:human
    character:animal
    character:cartoon
    character:crowd
    character:cinematic

intensity:
  How intrusive or strong the sound is.
  Values:
    intensity:low
    intensity:medium
    intensity:high

style (optional):
  Aesthetic or comedic category.
  Examples:
    style:fart
    style:impact
    style:ui

rating (optional):
  Content suitability.
  Examples:
    rating:clean
    rating:rude
```
--------------------------------------------------
DESIGN PHILOSOPHY
--------------------------------------------------

- Files contain no meaning; metadata does.
- Tags are cheap; rigidity is expensive.
- Missing matches are signals, not bugs.
- Adding sounds requires no code changes.

--------------------------------------------------
EXAMPLE QUERIES
--------------------------------------------------

Play a happy success sound:
```python
  bank.pick(["event:success", "mood:happy"])
  ```

Play a clean sarcastic reaction:
```python
  bank.pick(["intent:sarcasm", "rating:clean"])
```
Play a low-intensity idle sound:
```python
  bank.pick(["intensity:low", "mood:curious"])
```

--------------------------------------------------
EXTENDING THE SCHEMA
--------------------------------------------------

You may add new tags at any time:
```yaml
  context: night
  environment: outdoors
  audience: kids
  language: nonverbal
```

Unknown tags are ignored unless queried.

--------------------------------------------------
SUMMARY
--------------------------------------------------

This soundbank enables:
- Emotion-driven audio
- Context-aware reactions
- Clean separation of data and behavior
- Easy growth without refactoring

Your robot does not just play sounds.
It expresses intent.