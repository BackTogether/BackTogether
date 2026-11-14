"""Hear one of a few words (yes / no / help...) offline with Vosk, for the care flow.

Runs on the robot (system Python 3.8). One-off setup there (good wifi, ~70 MB, not in git):

    python3 -m pip install --user vosk==0.3.45
    cd ~ && wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip ...

The recogniser only knows the words it is asked for (a grammar), which makes it far more
reliable than free dictation. Audio comes from the head-camera microphone through `arecord`;
set LISTEN_AUDIO_COMMAND to read from somewhere else (another mic, or a file for tests).
"""

import json
import logging
import os
import select
import shlex
import subprocess
import threading
import time

SAMPLE_RATE = 16000
CHUNK_BYTES = 4000  # 0.125 s of 16-bit mono audio
DEFAULT_MODEL_DIR = os.path.expanduser("~/vosk-model-small-en-us-0.15")
MIC_CARD = "A681600W"  # USB microphone of the head camera (`arecord -l` on the robot)
DEFAULT_AUDIO_COMMAND = (
    f"arecord -q -D plughw:CARD={MIC_CARD},DEV=0 -f S16_LE -r {SAMPLE_RATE} -c 1 -t raw"
)
UNKNOWN_WORD = "[unk]"
MIN_CONFIDENCE = 0.5  # per-word confidence from Vosk, 0..1

log = logging.getLogger("listen")
_model = None
_model_lock = threading.Lock()


def available() -> bool:
    """True if Vosk and its model are installed, so wait_word() can work."""
    try:
        import vosk  # noqa: F401  (only installed on the robot)
    except ImportError:
        log.warning("vosk not installed: answering out loud is off")
        return False
    if not os.path.isdir(_model_dir()):
        log.warning("vosk model not found at %s: answering out loud is off", _model_dir())
        return False
    return True


def wait_word(options: dict, timeout_s: float, cancel=None):
    """Listen until one of the words is heard. `options` maps each answer to the words that mean
    it, e.g. {"ok": ["yes", "fine"], "help": ["help", "no"]}. Returns the answer, or None on
    timeout, when `cancel` (a threading.Event) is set, or if listening is not possible."""
    word_to_option = {w: option for option, words in options.items() for w in words}
    recognizer = _recognizer(sorted(word_to_option))
    if recognizer is None:
        return None
    command = shlex.split(os.environ.get("LISTEN_AUDIO_COMMAND", DEFAULT_AUDIO_COMMAND))
    try:
        audio = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError as error:
        log.error("cannot start %s: %s", command[0], error)
        return None
    try:
        return _listen(audio.stdout, recognizer, word_to_option, time.time() + timeout_s, cancel)
    finally:
        audio.kill()
        audio.wait()


def _listen(stream, recognizer, word_to_option: dict, deadline: float, cancel):
    while time.time() < deadline and not (cancel and cancel.is_set()):
        ready, _, _ = select.select([stream], [], [], 0.2)
        if not ready:
            continue
        chunk = os.read(stream.fileno(), CHUNK_BYTES)
        if not chunk:
            return _match(json.loads(recognizer.FinalResult()), word_to_option)
        # Only final results (after a short pause): partial ones are forced onto the grammar
        # and turn unrelated speech into "help".
        if recognizer.AcceptWaveform(chunk):
            heard = _match(json.loads(recognizer.Result()), word_to_option)
            if heard:
                return heard
    return None


def _match(result: dict, word_to_option: dict):
    for word in result.get("result", []):
        if word["word"] in word_to_option and word["conf"] >= MIN_CONFIDENCE:
            log.info("heard '%s' (confidence %.2f)", word["word"], word["conf"])
            return word_to_option[word["word"]]
    return None


def _recognizer(words):
    model = _load_model()
    if model is None:
        return None
    import vosk

    grammar = json.dumps(list(words) + [UNKNOWN_WORD])
    recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE, grammar)
    recognizer.SetWords(True)  # per-word confidence
    return recognizer


def _load_model():
    """Load the model once (a few seconds), on first use."""
    global _model
    with _model_lock:
        if _model is None and available():
            import vosk

            vosk.SetLogLevel(-1)
            _model = vosk.Model(_model_dir())
        return _model


def _model_dir() -> str:
    return os.environ.get("VOSK_MODEL_PATH", DEFAULT_MODEL_DIR)
