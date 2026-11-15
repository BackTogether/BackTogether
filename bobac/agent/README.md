# bobac/agent: voice agent that talks with the lost child while waiting for the parent

Gemini Live (Google, native audio) keeps the child calm and in place from the moment the
robot calls the parent until the parent joins the video call. Then it stops at once, so it
never talks over the parent.

| File | What |
|---|---|
| `voice_agent.py` | the agent: Python 3.11 + `google-genai`, microphone with `parec`, voice with `pacat` |
| `child_prompt.txt` | its instructions; `{child_name}`, `{child_age}`, `{parent_called}`, `{emergency_number}` come from `bobac/config/child.json` |
| `../robot/agent_process.py` | starts / stops it from the care flow (the robot's Python 3.8 cannot run the SDK) |

## How it fits the care flow
`care.py` starts the agent as soon as the robot reaches the child (it says the first hello and
replaces the robot's own voice), and stops it when `parent_in_call` turns `True` (the parent
joined), on `/stop`, and when the episode ends. Without `GEMINI_API_KEY`, or if the agent is
not installed, the robot speaks with its own voice as before.

## Setup (once)
1. **API key:** log into [AI Studio](https://aistudio.google.com) with the hackathon `@gcplab.me`
   account → *Get API key*. If none is listed: *Import Projects* → the only project. Never commit
   it, never paste it in an issue or chat (the organisers shut down leaked keys).
2. **Install on the robot, offline** (portable Python 3.11 + `google-genai`, no sudo): copy the
   bundle `notes/robot_bundle/gemini_agent/` (local, not in git: ~40 MB) to the robot and run
   `bash install.sh` there. It creates `~/leuk/python` and `~/leuk/agent-venv`.
3. **Code:** copy this folder to `~/leuk/agent/` (next to `~/leuk/robot/`).

## Try the agent alone (robot, no call)
```bash
export GEMINI_API_KEY='<key>'
~/leuk/agent-venv/bin/python ~/leuk/agent/voice_agent.py --child-name Leo --child-age 7 \
    --parent-called Mom --emergency-number 112
```
It greets the child and listens. `Ctrl+C` stops it. Its log shows what it heard and said.

## Full flow
```bash
export GEMINI_API_KEY='<key>' NTFY_TOPIC='<topic>'
cd ~/leuk/robot && python3 care_server.py
```
The agent's own log goes to `~/leuk/voice-agent.log`.

## Settings (environment)
| Variable | Default |
|---|---|
| `GEMINI_LIVE_MODEL` | `gemini-2.5-flash-native-audio-preview-12-2025` |
| `GEMINI_VOICE` | `Puck` (other prebuilt voices: Aoede, Kore, Charon...) |
| `GEMINI_API_VERSION` | `v1alpha` (the SDK default answers 404 to the Live connection) |
| `VOICE_AGENT_PYTHON` | `~/leuk/agent-venv/bin/python` |

## Known risks
- **Echo:** the robot's speaker reaches its microphone and the agent may hear itself. Turn on
  the sound server's echo cancellation (no sudo; lost on reboot):
  `pactl load-module module-echo-cancel aec_method=webrtc source_name=ec_mic sink_name=ec_speaker`
  then `pactl set-default-source ec_mic && pactl set-default-sink ec_speaker`.
- **Network:** live audio both ways. On the robot's slow wifi expect pauses; a phone hotspot helps.
- **Quiet speaker:** a USB speaker is plug and play and much louder.
- The hackathon Google environment is deleted when the event ends.
