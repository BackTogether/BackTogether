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
