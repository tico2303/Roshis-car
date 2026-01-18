import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# mic to text


DARO_VARIANTS = ["darrow", "dario", "darko", "darryl", "dora", "narrow", "doral", "darling", "doro", "door row", "darren", "dara", "dar row", "dar round", "durrani"]


MODEL_PATH = "/home/pi/Code/Roshis-car/pi/app/models/vosk-model-small-en-us-0.15"
SAMPLE_RATE = 16000

q: "queue.Queue[bytes]" = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

model = Model(MODEL_PATH)
rec = KaldiRecognizer(model, SAMPLE_RATE)

# If your .asoundrc is set, you can omit device=...
with sd.RawInputStream(
    samplerate=SAMPLE_RATE,
    blocksize=8000,
    dtype="int16",
    channels=1,
    callback=callback,
):
    print("Listening... (Ctrl+C to stop)")
    while True:
        data = q.get()
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "").strip()
            if text:
                print("FINAL:", text)
        else:
            partial = json.loads(rec.PartialResult()).get("partial", "").strip()
            if partial:
                print("...", partial)