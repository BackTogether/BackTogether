# drone/docker: run the drone web app, the lost-child tools and the patrols

Two images, built from `drone/` with `Dockerfile` and `compose.yaml` in this folder:

| Image | What | Size |
|---|---|---|
| `leukerbad/drone-app` | hoverctl web UI + `drone.sdk` HTTP API + simulator, and the tools that need no depth model: `drone/find`, `drone/land`, `drone/dock`, `drone.identify` (Python 3.11, bleak, ffmpeg, numpy, OpenCV) | ~1.1 GB |
| `leukerbad/drone-patrol` | the app image + the patrols (bounce, dvd, clear, sweep, so also `drone/find --sweep`) and front distance, with CPU torch and the Depth Anything V2 Metric Indoor Small model **baked in** (works offline on the drone's Wi-Fi) | ~3 GB |

The launchers work unchanged inside the images: they call `uv run ... python ...`, and the
images carry a tiny `uv` that just runs what follows (the dependencies are already installed).

**Never in the images** (`drone/.dockerignore`): the photos of people (`identify/people`,
`base`, `body`, `robot`) and the identify models (`identify/models`). The `tools` service mounts
`identify/people`, `identify/models` and `recordings/` from the repo, so what you trained or
downloaded on the host is what the container uses, and nothing personal travels with an image.

## Where each piece runs
The web app reaches the drone over **Bluetooth** (state, Wi-Fi start, MANUAL) and the drone's
**Wi-Fi** (sticks to UDP 19091, video pushed to our UDP 7078 at the IP we announce). Docker on
macOS and Windows runs in a Linux VM: no Bluetooth, and an IP the drone cannot reach. So:

| Host | Drone link (web app + SDK) | Front distance + patrol |
|---|---|---|
| **Windows** | native: `drone\fly` (cmd / PowerShell), or `drone/patrol/winfly web` from WSL | native: `drone\patrol\live`, or Docker, profile `patrol` → `host.docker.internal:8077` |
| **macOS** | native: `drone/fly` | Docker, profile `patrol` → `host.docker.internal:8077` |
| **Linux** | Docker, profile `drone` (host network + host BlueZ) | Docker, `--network host` (see the end) |
| any, no drone | Docker, profile `sim` | Docker, profile `sim` |

Verified on Windows 10 + Docker Desktop (2026-11-14): the sim profile end to end, the patrol
container driving a native SDK through `host.docker.internal`, the native link starting with
Bluetooth. Not verified yet: the Linux `drone` profile.

## 1. One-time setup (with internet)
Windows (the team laptop already has all of this):
1. **Docker Desktop** running.
2. **uv** (`powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`) and **ffmpeg**
   (`winget install Gyan.FFmpeg`; open a new terminal afterwards).
3. A Windows clone of the repo at `%USERPROFILE%\leukerbard-win` (`winfly` mirrors `drone/` into it).
4. Build the images and cache the link's only dependency (bleak), **before** joining the
   drone's Wi-Fi, which has no internet:
   ```bash
   drone/patrol/winfly sim        # builds both images and opens the simulator (step 2)
   ```
   or in PowerShell: `cd drone\docker; docker compose --profile sim build; docker compose --profile patrol build`.

macOS: Docker Desktop, `brew install uv ffmpeg`, then build as above.

## 2. Try it without the drone (any OS)
```bash
cd drone/docker
docker compose --profile sim up --build
```
Open http://localhost:8077: the simulated drone takes off by itself and bounces off the walls
(4 m box). The terminal prints one line per reading: `FORWARD front= 2.19 m`, `-> HOVER (wall
at 0.98 m)`, `-> TURN -145 deg`. Stop with `Ctrl+C`, clean up with `docker compose --profile sim down`.

### The lost-child mission and the landing on the robot
```bash
cd drone/docker
docker compose --profile mission up --build --abort-on-container-exit
```
Open http://localhost:8077. The simulator has a fake child (3.5 m ahead, 1.5 m left) and a
fake detector. The `mission-sim` container enters MANUAL, starts the test mission
(`drone/patrol/live mission`) and presses the takeoff button: the drone patrols, finds the
child, fixes its position from overhead and orbits it. When the mission turns for home,
`drone/dock --sim` takes over and lands the drone on the pad of a simulated robot standing at
`SIM_ROBOT` (takeoff frame, default `3 1`). About 2 minutes; it ends with `landed`. Verified on
WSL2 + Docker Desktop (2026-11-15).

## 3. Fly the real drone (Windows, from a WSL terminal at the repo root)
Without WSL and Docker, everything runs natively from `cmd`: see "Operating systems" in
[drone/sdk/README.md](../sdk/README.md) (`drone\setup`, `drone\fly`, `drone\patrol\live`).

Before: close the **Hover app** on the phone (the drone takes one Bluetooth client), drone
charged (> 60 %), and if you need internet while on the drone's Wi-Fi, turn on the phone's
**USB tethering**.

1. **Drone link + web page:** `drone/patrol/winfly web` (a PowerShell window opens; leave it).
   Same thing in PowerShell: `uv run --no-project --python 3.11 --with bleak python -m drone.sdk serve --http-port 8077 --no-browser --max 60`.
   If Windows asks about the **firewall**, allow Python on private and public networks (video).
2. **Switch the drone on** (hold the big button ~2 s) and open **http://127.0.0.1:8077**.
   The DRONE panel should show the Bluetooth link, battery, and `wifi: on · HoverX1_060039`
   (the app turns the drone's Wi-Fi on by itself, password `<DRONE_WIFI_PASSWORD>`).
3. **Join the drone's Wi-Fi:** `drone/patrol/winfly wifi`, or pick `HoverX1_060039` in the
   Windows Wi-Fi menu with password `<DRONE_WIFI_PASSWORD>`. The laptop loses internet unless it is tethered.
4. **MANUAL CONTROL** in the page: wait for the check marks and the **video**.
5. **Take off with the drone's button** from the palm. Keys: W/S forward/back, A/D left/right,
   I/K up/down, J/L turn, U/O camera, Space = stop sticks, **LAND = double click**.
6. **Tilt the camera down** with the tilt preset buttons (the patrol uses -30° by default).
7. **Front distance only:** `drone/patrol/winfly distance --takeoff-height 1.0`.
8. **Patrol, decisions only:** `drone/patrol/winfly patrol` (default `--dry-run`: prints what it
   would do, sends nothing).
9. **Patrol, flying:** `drone/patrol/winfly patrol --stop-at 1.5 --speed 0.3`.
10. **Stop:** STOP or LAND in the page (STOP ends the patrol), `Ctrl+C` in the patrol window.
    Afterwards `drone/patrol/winfly home` rejoins the usual Wi-Fi.

### Find the child, land on the robot (containers, from WSL)
With the link running on Windows (steps 1-5), the launchers run in the `tools` container
(patrol image) from `drone/docker`. They default to `http://127.0.0.1:8077`, which inside a
container is not the host: pass `--url` (find, land, dock); `drone/patrol/live` reads `SDK`,
which the service already sets.
```bash
URL=http://host.docker.internal:8077
docker compose run --rm tools python -m drone.identify.models      # once, with internet: OpenCV Zoo models
docker compose run --rm tools drone/find leo --watch --url $URL    # no flight: recognise on the live video
docker compose run --rm tools drone/find leo --sweep --url $URL    # patrol, find, hover over the child
docker compose run --rm tools drone/land --watch --url $URL        # is the pad seen?
docker compose run --rm tools drone/dock --now --url $URL          # land on the robot's pad
docker compose run --rm tools drone/patrol/live status             # any `live` mode
```
`leo` is a folder of face photos in `drone/identify/people/` on the host. `drone/land` and
`drone/dock` need the pad model in `drone/identify/models/` (`python -m drone.identify.pad
train`, also runnable in the container). Ctrl+C stops the tool and the drone hovers, as natively.
Not verified with the real drone yet: only the simulator ran these tools in containers.

## 4. Test plan: is the patrol safe?
Do them in order; do not go on if one fails. One person on LAND, another on STOP.

| # | Test | Pass when |
|---|---|---|
| T0 | `winfly sim` | bounces in the simulator, never `wall_hits` |
| T1 | Hover at ~1.5 m, camera at -30°, in front of tape marks at **3, 2 and 1 m** from a wall; run `winfly distance` | printed `front=` within ±0.3 m of each mark, `floor scale` near 1 |
| T2 | Fly by hand towards the wall with `winfly patrol` (dry-run) | prints `-> HOVER (wall at …)` before 1.5 m, then `-> TURN`, then `-> CHECK` / `-> FORWARD` when it faces open space |
| T3 | `winfly patrol --stop-at 1.5 --speed 0.3` from the middle of the room (≥ 3 m from walls) | stops ≥ 0.5 m from the wall (it drifts ~0.5 m while braking), turns, goes on; 3-5 minutes without touching anything |

If T1 is off by a constant factor, fix `--hfov` (horizontal field of view of the preview) or the
tilt; if it is off only close to the wall, raise `--stop-at`.

## Troubleshooting
| Symptom | Cause / fix |
|---|---|
| `drone not found (powered on, Hover app closed?)` | drone off, or the phone app holds the Bluetooth link: force-stop the app |
| `ModuleNotFoundError: termios` on Windows | old `teleop.py`: pull; termios is imported only by the terminal teleop now |
| `Failed to download mujoco…` | use `--no-project` (the link needs only bleak) |
| `ffmpeg not found` | `winget install Gyan.FFmpeg`, then a **new** terminal |
| no video after MANUAL | laptop not on `HoverX1_…`, or the firewall blocked Python (UDP 7078) |
| patrol: `Connection refused` / unreachable | the native link is not running on 8077; do not add `extra_hosts` (it overrides Docker Desktop's `host.docker.internal`) |
| `HOLD: …` lines in the patrol | a safety stop: not flying in MANUAL, below `--min-alt`, camera tilt unknown, or no fresh distance |

## Linux host (not verified yet)
The `drone` service runs the link in the container with the host network (Wi-Fi, UDP) and the
host's BlueZ over D-Bus; join the drone's Wi-Fi on the host (nmcli) first:
```bash
docker compose --profile drone up --build
docker compose run --rm --network host patrol python -m drone.patrol.bounce --sdk http://127.0.0.1:8077 --dry-run
```
