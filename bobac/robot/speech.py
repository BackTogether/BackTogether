"""The robot's voice (P3 in interfaces/ROBOT_API.md): fixed phrases, free text and audio files.

Runs on the robot (system Python 3.8). Speaks through `spd-say` (speech-dispatcher, already
installed there) and plays files with ffmpeg + aplay. Where those tools are missing (a laptop)
nothing sounds: the text is logged and the call returns False, so flows can still be tried.
"""

import logging
import shutil
import subprocess
import threading

PHRASES = {
    # ids fixed by the contract (interfaces/ROBOT_API.md)
    "greeting": "Hi! I am a friendly robot. I am here to help you.",
    "are_you_ok": "Are you okay?",
    "press_button": "Say yes, or press the green button, if you are okay. "
    "Say no, or press the red button, if you are not.",
    "all_good": "Great. You are very brave.",
    "calling_help": "I am calling your family now.",
    "help_on_way": "Your family is on the way. I am staying with you.",
    # used by the care flow (care.py)
    "did_you_get_hurt": "Did you get hurt? Say yes or no, or press a button.",
    "sorry_hurt": "I am sorry you got hurt. I will tell your family right away.",
    "stay_here": "Please stay here with me. Do not walk away.",
    "still_here": "I am still here with you. You are not alone.",
    "count_intro": "Let us count to five together. Show me your fingers!",
    "count_done": "Well done! You are doing great.",
}
LANGUAGE = "en"
SPEECH_RATE = -20  # spd-say scale -100..100; slower than default, easier to follow
SPEECH_VOLUME = 100  # spd-say scale -100..100; the screen speaker is quiet, so always max
SAY_TIMEOUT_S = 40
PLAY_TIMEOUT_S = 120

log = logging.getLogger("speech")
_one_voice_at_a_time = threading.Lock()


def say(phrase_id: str) -> bool:
    """Speak a fixed phrase and wait until it ends. False if the id is unknown or nothing played."""
    text = PHRASES.get(phrase_id)
    if text is None:
        log.error("unknown phrase id: %s", phrase_id)
        return False
    return say_text(text)


def say_text(text: str) -> bool:
    """Speak any text and wait until it ends."""
    log.info("say: %s", text)
    if shutil.which("spd-say") is None:
        log.warning("spd-say not installed: nothing was spoken")
        return False
    command = [
        "spd-say", "--wait", "-l", LANGUAGE, "-r", str(SPEECH_RATE), "-i", str(SPEECH_VOLUME), text
    ]  # fmt: skip
    return _run_exclusive(command, SAY_TIMEOUT_S)


def play_file(path: str) -> bool:
    """Play an audio file of any format ffmpeg reads (e.g. a Telegram voice note)."""
    log.info("play: %s", path)
    if shutil.which("ffmpeg") is None or shutil.which("aplay") is None:
        log.warning("ffmpeg or aplay not installed: nothing was played")
        return False
    decode = ["ffmpeg", "-loglevel", "error", "-i", path, "-f", "wav", "-"]
    with _one_voice_at_a_time:
        decoder = subprocess.Popen(decode, stdout=subprocess.PIPE)
        try:
            player = subprocess.run(
                ["aplay", "-q"], stdin=decoder.stdout, timeout=PLAY_TIMEOUT_S, check=False
            )
        except subprocess.TimeoutExpired:
            log.error("playing %s took too long", path)
            return False
        finally:
            decoder.kill()
            decoder.wait()
    return player.returncode == 0


def _run_exclusive(command, timeout_s: float) -> bool:
    with _one_voice_at_a_time:
        try:
            result = subprocess.run(command, timeout=timeout_s, check=False)
        except subprocess.TimeoutExpired:
            log.error("speech command took too long")
            return False
    return result.returncode == 0
