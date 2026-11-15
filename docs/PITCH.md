# Pitch notes: BackTogether

Working notes for the 3-4 minute pitch to a mixed jury (founders, engineers, non-technical
judges). What we defend, the use cases, the demo, and the technical work worth showing off.
Items marked **[verify]** need a check before they go on a slide or are said out loud.

## 1. The one-liner
> **A drone finds the person. A robot stays with them until their family arrives.**

Positioning: **find and reach vulnerable people in public spaces**. Lost child is the demo;
the same pipeline covers anyone a family or a care team is looking for.
Not "a smart city platform" (too vague), not "a lost-child gadget" (too narrow).

Name in the product: **BackTogether** (the parent web app). Slide title so far: *Find My Child*.
Pick one for the pitch and use it everywhere.

## 2. The problem
- In crowded public places (events, ski resorts, thermal baths, markets, train stations) people
  get separated, and the first minutes matter most.
- Today the search is people walking around with a description and a phone. Slow, it does not
  scale with the area, and the person found is often scared, alone, or hurt.
- Finding is only half of it: someone has to **stay with them** and **reach the family**.
- **[verify]** One or two real numbers would help (lost children at large events, missing
  people with dementia per year in Switzerland). Do not quote a figure we have not sourced.

## 3. Our approach: find from the air, care on the ground
1. **The family sends a photo** from their phone (the BackTogether web app: name + photos, nothing
   else, no account).
2. **The drone searches** the area autonomously and recognises *that* person in its live video.
3. **It locates them on the robot's map** and sends the position to the ground robot.
4. **The robot drives there**, avoiding obstacles, turns until it sees the person's face.
5. **The robot takes care of them**: a friendly voice agent talks to them, the family gets a push
   notification and a **video call opens on the robot's screen**.
6. **The drone flies back and lands on the robot**, ready for the next search.

### Why two robots and not just a drone
- The drone is fast and sees everything from above, but its battery lasts minutes and it cannot
  stay at eye level talking to a scared child.
- The ground robot is slow and sees little, but it can stay as long as needed, talk, show a
  face, hold a call and carry things (blanket, first-aid kit, water).
- **Each one does what the other cannot.** And the drone lives on the robot: it takes off from
  it and lands back on it (the *marsupial* concept: a carrier that brings the scout to the area,
  collects it and could recharge it).

## 4. Use cases
### Hero (the live demo): a lost child at an event
Parent sends a photo → drone finds the child → robot goes to them, calms them down with a voice
agent → video call with the parent until they arrive. Easy for everyone in the room to picture.

### Same pipeline, more impact: a person with dementia who wandered off
- Same input (a photo from the family or the care home), same search, same robot.
- High risk: getting lost outdoors, cold weather in the mountains.
- The robot fits even better here: calm voice, a familiar face on the screen, and no uniformed
  stranger approaching.
- **Best second example**: it proves this is not a one-off demo but a pipeline.

### Extensions (pitch only, not in the demo)
- **Person down**: the drone spots someone lying still (pose detection, no photo needed); the
  robot checks whether they answer and calls emergency services with position and video.
  (We started this: the early fall-detection patrol branch.)
- **Lost pet**: same photo-based search, the owner joins the call.
- **Sweep after an evacuation** (flood, landslide, fire): the drone checks an evacuated area for
  anyone left; the robot brings information or a kit without sending a rescuer in.

### Who would buy it (for the founders in the jury)
Event organisers, ski resorts, thermal baths and theme parks, train stations, municipalities and
care homes. Model: robot + drone as a service for an event or a season, with the operator's own
staff in the loop. **[verify]** Keep this to one sentence unless asked.

### What we do not claim
- Not mass surveillance and not a face database (see section 7).
- Not mountain rescue in open terrain: the robot has wheels.
- Not a replacement for staff or emergency services: it buys time and keeps the person safe and
  connected until a human arrives.

## 5. What the jury sees (demo flow)
1. A parent fills the BackTogether form on a phone (name + photo).
2. The drone takes off from the robot's back, sweeps the hall and recognises the child.
3. It approaches, centres straight above the child, takes a position fix, and sends it.
4. The robot drives there, turns to find the child's face, stops.
5. The voice agent greets the child by name; the parent's phone gets a *Join the video call*
   notification; the call opens on the robot's screen.
6. The parent hangs up (*family arrived*); the drone returns and lands on the robot's pad.

Keep a recorded backup video of the full run in case the live one fails.

## 6. What we built (technical brag list)
Lead with the first three for a technical jury; for a non-technical one, say "we taught a
consumer selfie drone to fly itself and search for a person".

### Reverse-engineered a closed consumer drone
- The HoverAir X1 only works with its phone app: **no SDK, no API**.
- We decompiled the Android app and captured its traffic to rebuild the whole protocol:
  Bluetooth LE commands and telemetry (protobuf over ADGP framing), 30 Hz UDP stick control,
  Wi-Fi RPC, and the live RTP/H.264 video (1280×720, 30 fps).
- Result: a laptop flies the drone, streams its video and reads its telemetry, **no phone**.
- On top of it we wrote a full **Python SDK**: telemetry and video streams, navigator (goto,
  turn, climb, waypoint patrols), photos and MP4 recording, an HTTP API, a ROS client, and a
  simulator with the same interface.

### Autonomy with almost no sensors
The X1 gives us one camera and position at **1 Hz** in a frame that resets at every takeoff:
no compass, no heading, no attitude, no gimbal angle. We built what was missing:
- **Heading from the video** (optical flow, phase correlation) fused with a stick model and the
  direction of travel.
- **Camera tilt estimator** from the gimbal commands, calibrated on our drone (the drone never
  reports its tilt).
- **Camera intrinsics** fitted from 360° spins.
- **Distance to walls from a single camera**: a monocular depth model (Depth Anything V2) plus
  where the floor ends in the image, turned into metres with camera height and tilt. Within
  0.1 m (median) on a measured wall approach.
- **Safety layer**: STOP always wins, geofence, altitude limits, low battery → fly home and
  land, link or video lost → hover, then land.

### Search patterns that cover the whole area
- **Lawnmower sweep**: parallel lanes with spacing computed from the camera footprint and
  overlap, live coverage map, and a revisit of the biggest unseen patches at the end.
- Simulator, 30 × 20 m hall: **100 % coverage in ~11 min, zero wall hits**, also with heading
  errors injected. (Plus bounce, reflection and wall-following patrols we compared it with.)

### Recognising one specific person from the air
- Offline, CPU only, open models: **YuNet** (faces) + **SFace** (face match against the
  family's photos) + **YOLOX** (whole bodies). About 60 ms per frame on a laptop.
- Face first, then **body tracking** (clothes colour + tracker), because at the end the camera
  looks straight down and no face is visible. A face of someone else drops the lock.
- Threshold calibrated on a video **without** the target, so it does not fire on strangers.

### Locating the child for the robot
- The drone approaches, puts the camera straight down, centres the child and takes a **position
  fix**, then converts it into the **robot's map frame** (the robot's pose at takeoff + an
  alignment leg).
- In the simulator: child placed within **0.12-0.24 m** of the truth. **[verify]** real-world
  error, if we measured it.

### Drone returns and lands on the robot
- Small trained detectors (HOG + colour + SVM) find the **robot** from afar and the **landing
  pad** on it up close. Robot detector: 88 % found, no false positive on 124 unseen frames.
- Waits for the right moment (robot arrived, or the parent hung up), flies back, lands on the pad.

### The ground robot (Bobac, ROS 1 Noetic)
- **SLAM + navigation** (gmapping, move_base with the TEB planner): it reaches a goal in a space
  it did not know, avoiding obstacles.
- `child_goal_server`: receives the child's position, drives to **0.8 m** short of them, facing
  them.
- On arrival it **rotates and searches for the face** with its own head camera, because the
  position has some error.
- A **state machine** (`home → arrived → searching → care → done`) on a latched ROS topic: every
  node knows where the demo is.
- All our robot code was moved into **ROS packages** (`bringup`, `kid_care`), modular and
  launched with one command.

### Care flow: talking to a scared child
- **Gemini Live speech-to-speech** voice agent with a child-safety prompt: short sentences,
  keeps them in place, never asks personal details, never tells them to go anywhere, escalates
  if they are hurt.
- Push notification to the parents (ntfy: no account, no phone number) with one *Join* button.
- **Jitsi video call** that opens full screen on the robot and joins by itself; the voice agent
  goes quiet when the parent joins.
- Graceful degradation: no API key → fixed phrases; any missing part only switches itself off.

### Connecting everything
- Drone laptop ↔ robot: HTTP endpoints over a direct Ethernet link, an iPhone USB link, or an
  **ngrok tunnel kept up from boot**, plus ntfy topics as a message bus. The robot listens all
  the time; any authorised host can follow the mission.
- We got into the robot platform without its credentials, through a tunnel, and recovered the
  password from its logs. **[verify]** exact wording with the team; say it briefly and in a
  "hacker spirit" tone, it can sound bad to some judges.
- **Runs natively on macOS, Windows and Linux** (Bluetooth, Wi-Fi joining, video per OS), so any
  operator laptop works.

### How we worked (only if asked)
Six people, each with a coding agent, in parallel: GitHub issues board, one branch per task,
reviewer agent, conventional commits enforced by hooks and CI.

## 7. Privacy and safety by design
Say this before the jury asks about face recognition:
- **We only search for the person the family asks us to find**, with their consent and their
  photos.
- No face database of the public: strangers are never identified, only compared and discarded.
- Models run **offline, on the laptop**: video and photos never go to a cloud. Photos stay local
  and out of git. **[verify]** We do not delete them automatically yet: say "are deleted after
  the search" only as the product rule, not as something the demo does.
- Safety: the drone has a geofence, altitude and battery limits and a STOP that always wins; the
  robot stops short of the child and never asks them to move.

One sentence for the pitch: *"We don't watch the crowd. We look for the one person a family asks
us to find."*

## 8. Honest limits and next steps
- The X1 takes off only with its button (it refuses remote takeoff); indoor demo, small hall.
- Position error grows with drift (1 Hz odometry, no compass); the robot's face search on arrival
  is how we absorb it.
- Face match needs faces of ~24 px or more: fly lower or closer.
- **[verify]** which steps ran on the real drone vs only in the simulator. Claim only what ran.
- Next: real charging on the robot's pad, outdoor GPS drone, several drones per robot, the person
  down use case, integration with the event's staff radio or the police.

## 9. Suggested structure (≈3:30)
| Time | Part | Content |
|---|---|---|
| 0:00-0:30 | Hook | A parent at a crowded event, their child is gone. What happens in the next 10 minutes? |
| 0:30-1:00 | Idea | One-liner. Find from the air, care on the ground. Why two robots |
| 1:00-2:15 | Demo | Live run (or video), narrated step by step |
| 2:15-3:00 | How | 3 brag points: reverse-engineered drone, autonomy with one camera and no compass, robot SLAM + voice agent + call |
| 3:00-3:20 | Beyond | Same pipeline: dementia, person down, after an evacuation. Privacy sentence |
| 3:20-3:30 | Close | One-liner again + team |

## 10. Likely questions
- **Why not just use CCTV?** Not everywhere, not where the child is now, and a camera cannot
  walk up to them and stay with them.
- **Why not just send a person?** We do not replace them: we find faster and keep the child safe
  and in contact while staff are on the way.
- **Face recognition and privacy?** Section 7.
- **What if the match is wrong?** Threshold calibrated against strangers, two consecutive frames,
  a second check by the robot's own camera before the care flow, and the parent sees the child
  on the call before anything else happens.
- **Battery?** The drone searches, the robot waits; the drone comes back to the robot when low.
- **Outdoors / scale?** Same software on an outdoor drone with GPS; the robot side is unchanged.

## 11. To do before the pitch
- [ ] Choose the name (BackTogether or Find My Child) and align slide + web app.
- [ ] Source one or two real numbers for the problem slide, or leave numbers out.
- [ ] Confirm what ran on the real hardware and update sections 6 and 8.
- [ ] Record the backup video of the full run.
- [ ] Decide who says what, and rehearse with a timer.
