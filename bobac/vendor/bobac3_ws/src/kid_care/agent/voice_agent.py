"""Gemini Live voice agent that keeps a lost child company until a parent joins the video call.

A separate program (Python 3.11+, google-genai) because the robot's own Python is 3.8. The care
flow starts it when the robot starts waiting for the parent and stops it (SIGTERM) as soon as
the parent joins the call (scripts/agent_process.py). Setup: the package README.

    GEMINI_API_KEY=... python voice_agent.py --child-name Leo --child-age 7 \\
        --parent-called Mom --emergency-number 112

Audio goes through the robot's sound server, so the video call can use the microphone too:
`parec` records the microphone, `pacat` plays the agent's voice.
"""

import argparse
import asyncio
import logging
import os
import signal
import subprocess
from pathlib import Path

from google import genai
from google.genai import types

DEFAULT_MODEL = "gemini-2.5-flash-native-audio-preview-12-2025"  # Live API, native audio
DEFAULT_VOICE = "Puck"  # a warm, friendly prebuilt voice
# The SDK's default API version answers 404 to the Live connection; v1alpha accepts it.
DEFAULT_API_VERSION = "v1alpha"
PROMPT_PATH = Path(__file__).with_name("child_prompt.txt")
MIC_RATE = 16000  # what the Live API expects as input
SPEAKER_RATE = 24000  # what the Live API sends back
CHUNK_BYTES = 3200  # 0.1 s of 16-bit mono microphone audio
AUDIO_LATENCY_MS = "80"
# Nobody talks first otherwise: this makes the agent greet the child as soon as it connects.
START_CUE = "The child is in front of you now. Greet them with your first message."

log = logging.getLogger("voice_agent")


def audio_command(tool: str, rate: int) -> list:
    return [tool, "--raw", "--format=s16le", f"--rate={rate}", "--channels=1",
            f"--latency-msec={AUDIO_LATENCY_MS}"]  # fmt: skip


class Speaker:
    """Plays the agent's voice; `flush()` drops what is queued when the child interrupts."""

    def __init__(self) -> None:
        self._process = None
        self._start()

    def play(self, pcm: bytes) -> None:
        try:
            self._process.stdin.write(pcm)
            self._process.stdin.flush()
        except BrokenPipeError:
            self._start()

    def flush(self) -> None:
        self.close()
        self._start()

    def close(self) -> None:
        if self._process:
            self._process.kill()
            self._process.wait()

    def _start(self) -> None:
        self._process = subprocess.Popen(
            audio_command("pacat", SPEAKER_RATE), stdin=subprocess.PIPE
        )


async def send_microphone(session) -> None:
    microphone = subprocess.Popen(audio_command("parec", MIC_RATE), stdout=subprocess.PIPE)
    try:
        while True:
            pcm = await asyncio.to_thread(microphone.stdout.read, CHUNK_BYTES)
            if not pcm:
                raise RuntimeError("the microphone stopped")
            await session.send_realtime_input(
                audio=types.Blob(data=pcm, mime_type=f"audio/pcm;rate={MIC_RATE}")
            )
    finally:
        microphone.kill()
        microphone.wait()


async def play_answers(session, speaker: Speaker) -> None:
    while True:  # receive() ends after every turn of the agent
        async for message in session.receive():
            content = message.server_content
            if content and content.interrupted:
                speaker.flush()  # the child spoke over the agent: stop the old sentence now
            if content and content.input_transcription and content.input_transcription.text:
                log.info("child: %s", content.input_transcription.text)
            if content and content.output_transcription and content.output_transcription.text:
                log.info("agent: %s", content.output_transcription.text)
            if message.data:
                await asyncio.to_thread(speaker.play, message.data)


async def talk(variables: dict) -> None:
    client = genai.Client(
        api_key=os.environ["GEMINI_API_KEY"],
        http_options={"api_version": os.environ.get("GEMINI_API_VERSION", DEFAULT_API_VERSION)},
    )
    voice = os.environ.get("GEMINI_VOICE", DEFAULT_VOICE)
    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        system_instruction=PROMPT_PATH.read_text(encoding="utf-8").format(**variables),
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice)
            )
        ),
        input_audio_transcription=types.AudioTranscriptionConfig(),
        output_audio_transcription=types.AudioTranscriptionConfig(),
    )
    model = os.environ.get("GEMINI_LIVE_MODEL", DEFAULT_MODEL)
    speaker = Speaker()
    try:
        async with client.aio.live.connect(model=model, config=config) as session:
            log.info("connected to %s", model)
            await session.send_client_content(
                turns=types.Content(role="user", parts=[types.Part(text=START_CUE)])
            )
            await asyncio.gather(send_microphone(session), play_answers(session, speaker))
    finally:
        speaker.close()


async def main_async(variables: dict) -> None:
    conversation = asyncio.create_task(talk(variables))
    loop = asyncio.get_running_loop()
    for stop_signal in (signal.SIGTERM, signal.SIGINT):  # SIGTERM: the parent joined the call
        loop.add_signal_handler(stop_signal, conversation.cancel)
    try:
        await conversation
    except asyncio.CancelledError:
        log.info("stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for name in ("child-name", "child-age", "parent-called", "emergency-number"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    if not os.environ.get("GEMINI_API_KEY"):
        parser.error("GEMINI_API_KEY is not set (AI Studio > Get API key)")
    asyncio.run(main_async(vars(args)))


if __name__ == "__main__":
    main()
