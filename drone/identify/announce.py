"""Tell the operator how the search ended: a sound, and the frame with the face marked."""

import math
import shutil
import struct
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import cv2

from .search import Sighting

SAMPLE_RATE = 22050
VOLUME = 0.6  # 0..1
BEEP_S = 0.18
GAP_S = 0.07
FOUND_TONES_HZ = (880, 1320)  # two rising beeps
FAILED_TONES_HZ = (220,)  # one low beep
# First player found is used: macOS, PulseAudio, ALSA, ffmpeg.
PLAYERS = (["afplay"], ["paplay"], ["aplay", "-q"], ["ffplay", "-nodisp", "-autoexit", "-v", "0"])
PLAY_TIMEOUT_S = 5
BOX_COLOUR = (0, 255, 0)  # BGR
BOX_THICKNESS = 3
LABEL_SCALE = 0.8


def _write_tones(path: Path, tones_hz: tuple) -> None:
    samples = []
    for tone in tones_hz:
        count = int(SAMPLE_RATE * BEEP_S)
        samples += [math.sin(2 * math.pi * tone * index / SAMPLE_RATE) for index in range(count)]
        samples += [0.0] * int(SAMPLE_RATE * GAP_S)
    with wave.open(str(path), "wb") as sound:
        sound.setnchannels(1)
        sound.setsampwidth(2)
        sound.setframerate(SAMPLE_RATE)
        sound.writeframes(b"".join(struct.pack("<h", int(s * VOLUME * 32767)) for s in samples))


def play(found: bool) -> None:
    """Two rising beeps when the person was found, one low beep when the search failed."""
    player = next((command for command in PLAYERS if shutil.which(command[0])), None)
    if player is None:
        sys.stdout.write("\a")  # terminal bell: better than silence
        sys.stdout.flush()
        return
    with tempfile.TemporaryDirectory() as folder:
        sound = Path(folder) / "beep.wav"
        _write_tones(sound, FOUND_TONES_HZ if found else FAILED_TONES_HZ)
        subprocess.run([*player, str(sound)], timeout=PLAY_TIMEOUT_S, check=False)


def save_evidence(sighting: Sighting, name: str, path: Path) -> bool:
    """Copy of the frame with a box and the name on the face that matched."""
    image = cv2.imread(str(sighting.frame))
    if image is None:
        return False
    left, top, width, height = sighting.box
    cv2.rectangle(image, (left, top), (left + width, top + height), BOX_COLOUR, BOX_THICKNESS)
    label = f"{name} {sighting.similarity:.2f}"
    cv2.putText(
        image,
        label,
        (left, max(top - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        LABEL_SCALE,
        BOX_COLOUR,
        2,
    )
    return bool(cv2.imwrite(str(path), image))
