# kid_care: stay with the lost child

Our ROS package for what the robot does **once it is with the child**: a Gemini voice agent
talks with them while the parents get a push notification with one *Join the video call*
button, and the call opens full screen on the robot's screen.

It starts by itself when the machine enters the **`care`** state — where `bringup`'s
`face_check` puts it the moment it recognises the child.

```bash
roslaunch bringup state_demo.launch     # home -> arrived -> searching -> care -> this package
roslaunch kid_care kid_care.launch      # this package alone, still waiting for the care state
```

## What happens

| Node | What it does |
|---|---|
| `kid_care` | the care flow (`scripts/care_server.py`): the robot's voice, the voice agent, the video call window, the message to the parents. Up from the start, idle until it is asked to begin |
| `kid_care_start` | waits for `/system_state` to be `care`, then hides the full screen image, frees the head camera and begins the episode |

Then, by itself:

1. the **voice agent** greets the child by name and keeps them company (`agent/voice_agent.py`,
   Gemini Live, prompt in `agent/child_prompt.txt`);
2. the **video call** opens full screen on the robot's screen, in a Firefox of its own, and
   presses *Join* for the child (Jitsi, a fresh room per episode). It is the only window the
   robot shows: there is no page of our own in front of it;
3. the **parents** get a push notification on `NTFY_TOPIC` with the call link;
4. when a parent joins, the agent goes quiet so they can talk; when they hang up, the robot
   takes it as *the family has arrived*, says goodbye and goes idle.

The two questions for the child (*are you okay?*, *did you get hurt?*) are off by default
(`ASK_QUESTIONS` in `scripts/care.py`): reaching the parents comes first.

## The keys: the hidden `.env`

The launch file reads **`kid_care/.env`** (hidden, never committed: `.env` is in the repo
`.gitignore`). `.env.example` is the same file without the keys.

```ini
GEMINI_API_KEY=your-key-from-ai-studio
NTFY_TOPIC='<NTFY_TOPIC>'
VOICE_AGENT_PYTHON=/home/bobac3/leuk/agent-venv/bin/python
```

Anything already exported in the shell wins over the file, so a quick test needs no edit:

```bash
GEMINI_API_KEY=other-key roslaunch bringup state_demo.launch
```

The parents subscribe to `NTFY_TOPIC` in the [ntfy](https://ntfy.sh) app, or just open
<https://ntfy.sh/<NTFY_TOPIC>> in a browser. No account, no phone number. Anyone who knows
the topic can read it, so use a long random one for anything but a demo.

Without `GEMINI_API_KEY` everything else still works: the robot greets the child with its own
fixed phrases instead of the agent.

## Launch arguments

| Argument | Default | What |
|---|---|---|
| `start_state` | `care` | the state that starts the episode; `""` starts it as soon as the server answers (no state machine) |
| `zone` | `park` | where the child is, as the parents are told |
| `port` | `8090` | the care server's HTTP port |
| `auto_start` | `true` | `false`: no trigger node, you POST `/care/start` yourself |
| `start_delay` | `2.0` | seconds between the trigger and the episode |
| `hide_splash` | `true` | hide `bringup`'s full screen image: it is always on top and would cover the call |
| `free_camera` | `true` | stop the nodes holding the head camera — the call's browser cannot open it otherwise |
| `env_file` | `$(find kid_care)/.env` | the hidden file with the keys |
| `child_profile` | `$(find kid_care)/config/child.json` | name, age, parents, emergency number |

```bash
roslaunch bringup state_demo.launch zone:="playground"   # passed through to this package
roslaunch bringup state_demo.launch care:=false          # stop at care, no agent and no call
roslaunch kid_care kid_care.launch start_state:=""       # this package alone, start now
rostopic pub -1 /system_state/set std_msgs/String "data: care"   # trigger it by hand
```

By hand, without any state:

```bash
roslaunch kid_care kid_care.launch auto_start:=false
curl -X POST localhost:8090/care/start -d '{"zone": "park"}'
curl localhost:8090/status
curl -X POST localhost:8090/stop
```

## `free_camera`: the call and `face_check` want the same camera

After `care`, `face_check` keeps following the child, and it needs `/head_camera/image_raw`.
The call's browser cannot open the head camera while a ROS node holds it, so
`kid_care_start` stops those nodes — which ends the following. The call wins by default;
`free_camera:=false` keeps the following and leaves the call without a camera.

## What it needs on the robot

ROS Noetic and the robot's system Python 3.8 (standard library + `rospy`), `bringup` built in
the same workspace (`kid_care_start` imports its `StateClient`), plus:

- **firefox** — the call window;
- **speech-dispatcher** (`spd-say`), **ffmpeg**, **alsa-utils** (`aplay`) — the robot's voice;
- **pulseaudio-utils** (`parec`, `pacat`) — the voice agent's audio, through the sound server so
  the video call can use the microphone too;
- **Python 3.11 with `google-genai`**, in its own venv — the voice agent
  (`python3.11 -m venv ~/leuk/agent-venv && ~/leuk/agent-venv/bin/pip install google-genai`),
  pointed at by `VOICE_AGENT_PYTHON`;
- **vosk** and its model — *optional*, lets the child answer out loud
  (`scripts/listen.py` says how).

Each missing piece only switches its own part off, with a warning in the log. Logs of the call
window and the agent: `~/leuk/call-firefox.log`, `~/leuk/voice-agent.log`.

## Files

```
launch/kid_care.launch      the care flow, started by the care state
scripts/kid_care_node.py    loads .env, runs the care flow
scripts/kid_care_start.py   waits for the care state, then begins the episode
scripts/care.py             what the robot does with the child, step by step
scripts/care_server.py      the HTTP API and the video call window
scripts/ntfy_channel.py     push notifications to the parents (or caregiver.py for Telegram)
scripts/speech.py           the robot's voice
scripts/listen.py           hearing yes / no / help offline (optional)
scripts/agent_process.py    starts and stops the voice agent
scripts/marionette.py       presses Join in the call window for the child
scripts/env_file.py         reads the hidden .env
agent/voice_agent.py        the Gemini Live agent (its own Python 3.11)
agent/child_prompt.txt      how it talks to a lost child
config/child.json           the child and their parents
```
