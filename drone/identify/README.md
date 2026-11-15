# drone/identify: find the lost child in the drone video

Open-source models from the OpenCV Zoo, offline, CPU only: YuNet finds faces, SFace compares
them with the reference photos, YOLOX-S finds whole people (the body tracker). About 60 ms per
1280×720 frame for all three on a laptop (M-series: faces ~12 ms, YOLOX ~40 ms).

## Set up (once, with internet)
```bash
mkdir -p drone/identify/people/leo                  # one folder per person (not in git)
cp photo_of_leo.jpg drone/identify/people/leo/      # clear, front-facing photos
uv run --with opencv-contrib-python-headless python -m drone.identify.models   # 75 MB of models
```
That also puts OpenCV (with contrib's CSRT tracker) in uv's cache, so from then on everything
runs without a network: add `--offline` to `uv run` (without it uv tries the index first).
Add a few other people too: a face that looks more like someone else is never taken for Leo.

Reference photos must be in `people/<name>/`: images directly in `people/` are not read.
Use sharp close-ups (phone photos): people.py ignores reference faces under 100 px. Drone frames
are not references: a small, blurry face is close to every other small, blurry face, so with
them strangers in the drone video reach the target's score (15 Nov 20:51 flight).
After changing the gallery, calibrate its threshold with a video **without the target**:
```bash
uv run --with opencv-contrib-python-headless python -m drone.identify.face_model demo \
    --negatives <video with other people>
```
This saves `people/demo/threshold.json` locally. Other galleries keep the default threshold
until calibrated. SFace's pretrained network is unchanged; the gallery and its threshold adapt
to the demo. From behind, where no face is visible, the body model below takes over.

## During the flight
Start it when the drone starts. It watches `recordings/frames/` (newest session), checks every
face the drone sees against the whole database and logs each one:
```bash
uv run --with opencv-python-headless python -m drone.identify          # everyone, until Ctrl+C
uv run --with opencv-python-headless python -m drone.identify leo      # only Leo, 120 s
```
- Whole database: each person seen in 2 frames in a row is announced once with **POSITIVE**,
  two rising beeps and `found_<name>.jpg` with the face marked.
- One person: stops at the first **POSITIVE** (exit code 0), or **FAILED** with one low beep
  (exit code 1) after `--timeout` seconds.

## Feeding the lost-child mission
`feed` reads the drone SDK server's frames (`GET /frame.jpg`) and, for each, posts the child to
the mission (`POST /mission/target`, drone/sdk/API.md) and everything it saw to the web page's
video overlay (`POST /vision`), with the frame's time:
```bash
uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed leo --url http://127.0.0.1:8077
```
It recognises the child by face (2 frames in a row), then tracks the **body**, because the
approach ends with the camera straight down where no face is visible (body_track.py): the person
box (YOLOX) around the face, matched every frame by overlap, size and clothes colour, with a
CSRT tracker for the frames the detector misses; the face re-confirms whenever it is seen, a
recognised face of someone else in the box drops the lock. The body is posted with anchor
`"feet"`: the mission puts its floor contact point on the floor, so `target_height_m` does not
matter. Until a body box is found around the face, the head is posted (anchor `"centre"`,
`target_height_m`). Confidence 0.9 face seen, 0.8 body found by the detector, 0.4 tracker only
(drawn on the page, below the mission's `min_confidence`).

On the page (drone/fly or the simulator): persons dashed grey, faces grey with their name, the
child's face white, the tracked body red; top left the lock state, the face match and the
mission's step. The video is delayed by the detector's latency (~0.1 s) so the boxes sit on the
frame they belong to; **B** turns the overlay (and the delay) off.

Test it on recorded frames (annotated copies in `--out`, one log line per frame):
```bash
uv run --offline --with opencv-contrib-python-headless python -m drone.identify.feed leo \
    --frames recordings/frames/<session> --first 40 --count 90 --out /tmp/annotated
```

## Body model (recognise the target without the face)
Phone photos of the target's clothes in `drone/identify/body/`, and actual drone photos at demo
height in `drone/identify/body/drone/` (not in git; in the phone photos the target is the
biggest person, in the drone photos the person dressed most like the phone photos).
The tracker (body_track.py) does not use it since the 15 Nov 20:53 flight: trained on one outfit,
it passed nearly every person in the hall and scored the target in other clothes lower than the
strangers around him. Only the face locks (and re-locks) on the target. To train it anyway:
```bash
uv run --with opencv-contrib-python-headless python -m drone.identify.body_model train \
    --negatives <video with other people, e.g. a drone video of the hall>
uv run --with opencv-contrib-python-headless python -m drone.identify.body_model check <image>
```
Use `--drone-photos <folder>` to override the drone folder. Landscape frames keep their aspect
ratio. Every original crop is included unchanged; real drone views receive mild light, blur,
scale and box changes. Phone photos use synthetic overhead views only when real drone photos
are unavailable.

The torso and leg colours feed an RBF SVM overfitted to the demo outfit. Training first reports
recall on held-out photos and augmentations, then fits **all** target photos and negative
crops. The final threshold accepts no fitted negative crop and is never below the SVM decision
boundary. Inspect `models/body_check.jpg` for wrong person boxes or rejected training photos;
`models/body_threshold.json` records the counts, validation results and parameters. Recognition
on fitted photos measures memorisation, rather than accuracy on a new flight.

The twelve current `photo_*.jpg` frames have been copied from `people/` into `body/drone/` and
`people/demo/`; originals are preserved. All photos, calibrated thresholds and model files stay
local and ignored by git. Run `drone/find demo --watch` to check the combined recognition on
the live video without starting a mission (requires `drone/fly` already running).

## Robot
`robot` finds the robot (Bobac) in drone frames from afar, to fly back to it before landing on
its pad (`pad`). Photos of the robot alone on the floor, seen diagonally, in
`drone/identify/robot/` (not in git); train once with a video of the hall without the robot:
```bash
uv run --with opencv-python-headless python -m drone.identify.robot train --negatives <video>
uv run --with opencv-python-headless python -m drone.identify.robot check <image or folder>
```
Non-floor blobs + HOG and colours + an SVM overfitted to this robot. 4 folds of held-out photos:
88 % found, no false positive in 124 unseen hall frames; ~30 ms per frame. `train` saves
`models/robot_check.jpg` with the robot box it found in each photo: check it.

## Landing pad
`pad` finds the landing pad on the robot (white square, grey circle, orange H) in drone frames,
seen diagonally from 0.6-4 m. Put a few photos of the pad in `drone/identify/base/` (not in git),
then train once (a few minutes; the video adds frames without the pad as negatives):
```bash
uv run --with opencv-python-headless python -m drone.identify.pad train --negatives <video>
uv run --with opencv-python-headless python -m drone.identify.pad check <image or folder>
```
`train` saves `models/pad_check.jpg`: check that the pad is marked in every photo. Set
`PAD_SIDE_M` in pad.py to the real side of the white square first.

## On recorded flights
```bash
uv run --with opencv-python-headless python -m drone.identify.survey recordings/frames
uv run --with opencv-python-headless python -m drone.identify leo --once \
    --frames recordings/frames/hover_video_20261114-182424
```
`survey` says per session how many frames show a face and how big the faces are: SFace needs
faces of about 24 px or more. If they are smaller, fly lower or closer.
