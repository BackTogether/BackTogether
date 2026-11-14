# drone/identify: find the lost child in the drone video

Open-source models from the OpenCV Zoo, offline, CPU only: YuNet finds faces, SFace compares
them with the reference photos. About 60 ms per 1280×720 frame on a laptop.

## Set up (once, with internet)
```bash
mkdir -p drone/identify/people/leo                  # one folder per person (not in git)
cp photo_of_leo.jpg drone/identify/people/leo/      # clear, front-facing photos
uv run --with opencv-python-headless python -m drone.identify.models   # 38 MB of models
```
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
`feed` reads the drone SDK server's frames (`GET /frame.jpg`) and posts the child to the mission
(`POST /mission/target`, drone/sdk/API.md) with each frame's time. It recognises the child by
face, then keeps tracking the head with an OpenCV tracker, because the approach ends with the
camera straight down where no face is visible:
```bash
uv run --with opencv-contrib-python-headless python -m drone.identify.feed leo --url http://<mac>:8077
```
The posted box is the head: start the mission with `target_height_m` = the child's head height
(about 1.0 standing, 0.6 sitting). Plain `opencv-python-headless` also works (MIL tracker, worse
than contrib's CSRT).

## On recorded flights
```bash
uv run --with opencv-python-headless python -m drone.identify.survey recordings/frames
uv run --with opencv-python-headless python -m drone.identify leo --once \
    --frames recordings/frames/hover_video_20261114-182424
```
`survey` says per session how many frames show a face and how big the faces are: SFace needs
faces of about 24 px or more. If they are smaller, fly lower or closer.
