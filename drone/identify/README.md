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
Start it when the drone starts: it watches `recordings/frames/` (newest session) and checks
every new frame the drone saves, logging each face it sees.
```bash
uv run --with opencv-python-headless python -m drone.identify leo            # live
uv run --with opencv-python-headless python -m drone.identify leo --timeout 300
```
- **POSITIVE**: two rising beeps, exit code 0, `found.jpg` with the face marked.
- **FAILED**: one low beep, exit code 1, after `--timeout` seconds without a confirmed match.

## On recorded flights
```bash
uv run --with opencv-python-headless python -m drone.identify.survey recordings/frames
uv run --with opencv-python-headless python -m drone.identify leo --once \
    --frames recordings/frames/hover_video_20261114-182424
```
`survey` says per session how many frames show a face and how big the faces are: SFace needs
faces of about 24 px or more. If they are smaller, fly lower or closer.
