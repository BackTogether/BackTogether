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
    # ids fixed by the contract
    "greeting": "Hello. I am your care robot. I am here with you.",
    "are_you_ok": "Are you okay?",
    "press_button": "Say yes, or press the green button, if you are fine. "
    "Say help, or press the red button, if you need help.",
    "all_good": "Good. I am glad you are fine.",
    "calling_help": "I am calling for help now. Stay calm.",
    "help_on_way": "Help is on the way. I am staying with you.",
    # used by the care flow
    "no_answer": "I did not get an answer, so I am calling for help.",
    "do_not_get_up": "Please do not try to get up quickly. Stay where you are.",
    "still_here": "I am still here with you. You are not alone.",
    "breathe_in": "Breathe in slowly.",
    "breathe_out": "And breathe out.",
    "help_arrived": "Help has arrived. You are in good hands.",
}
LANGUAGE = "en"
SPEECH_RATE = -20  # spd-say scale -100..100; slower than default, easier to follow
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
    command = ["spd-say", "--wait", "-l", LANGUAGE, "-r", str(SPEECH_RATE), text]
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
