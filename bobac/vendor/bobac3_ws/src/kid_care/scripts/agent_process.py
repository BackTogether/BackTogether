"""Start and stop the voice agent (the package's agent/voice_agent.py) from the care flow.

The agent needs Python 3.11 and google-genai, the robot's own Python is 3.8: it runs as a
separate program, in its own environment. Configuration, from the environment:

    GEMINI_API_KEY        the AI Studio key; without it there is no agent
    VOICE_AGENT_PYTHON    Python 3.11 that has google-genai (default ~/leuk/agent-venv/bin/python)
    VOICE_AGENT_SCRIPT    the agent program (default: ../agent/voice_agent.py next to this file;
                          the launch file sets it, so an installed package works too)
"""

import logging
import os
import subprocess
import threading
from pathlib import Path

DEFAULT_PYTHON = Path.home() / "leuk" / "agent-venv" / "bin" / "python"
DEFAULT_SCRIPT = Path(__file__).resolve().parent.parent / "agent" / "voice_agent.py"
LOG_PATH = Path.home() / "leuk" / "voice-agent.log"
STOP_TIMEOUT_S = 3

log = logging.getLogger("agent_process")
_lock = threading.Lock()
_process = None


def available() -> bool:
    """True if the agent can run here; logs what is missing otherwise."""
    if not os.environ.get("GEMINI_API_KEY"):
        log.info("GEMINI_API_KEY not set: no voice agent")
        return False
    for path in (_python(), _script()):
        if not path.exists():
            log.warning("voice agent not installed: %s missing (see the package README)", path)
            return False
    return True


def start(variables: dict) -> None:
    """Start talking with the child. `variables` fill the {...} in the agent's prompt."""
    global _process
    with _lock:
        if _process is not None and _process.poll() is None:
            return  # already talking
        command = [str(_python()), str(_script())]
        for name, value in variables.items():
            command += ["--" + name.replace("_", "-"), str(value)]
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(str(LOG_PATH), "ab") as output:
            _process = subprocess.Popen(command, stdout=output, stderr=output)
        log.info("voice agent started (log: %s)", LOG_PATH)


def stop() -> None:
    """Stop talking: the parent joined the call, or the episode is over."""
    global _process
    with _lock:
        if _process is None:
            return
        if _process.poll() is None:
            _process.terminate()
            try:
                _process.wait(STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                _process.kill()
                _process.wait()
            log.info("voice agent stopped")
        _process = None


def _python() -> Path:
    return Path(os.environ.get("VOICE_AGENT_PYTHON", str(DEFAULT_PYTHON)))


def _script() -> Path:
    return Path(os.environ.get("VOICE_AGENT_SCRIPT", str(DEFAULT_SCRIPT)))
