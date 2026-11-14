# drone/docker: the drone web app and the bounce patrol in containers

Two images, built from `drone/` (`Dockerfile`, `compose.yaml`):
- `leukerbad/drone-app`: the hoverctl web UI + `drone.sdk` HTTP API + simulator (stdlib, bleak, ffmpeg).
- `leukerbad/drone-patrol`: the bounce patrol (`drone.patrol.bounce`) with CPU torch and the
  Depth Anything V2 Metric Indoor Small model baked in, so it runs offline on the drone's Wi-Fi.

Run everything from this folder with Docker (Desktop) running:

| Profile | Command | Where it works |
|---|---|---|
| `sim` | `docker compose --profile sim up --build` | any OS: simulator + bounce patrol |
| `drone` | `docker compose --profile drone up --build` | **Linux host only**: the real drone |
| `patrol` | `SDK_URL=http://host.docker.internal:8077 docker compose --profile patrol up --build` | any OS, against an SDK running elsewhere |

Then open http://localhost:8077. The patrol prints its phase and the front distance on every
reading (`docker compose logs -f patrol-sim`). `Ctrl+C` or `docker compose --profile sim down` stops it.

## Why the real drone needs a Linux host (or no container)
The web app talks to the drone over **Bluetooth** (state, Wi-Fi start, MANUAL), then over the
drone's **Wi-Fi** (sticks to UDP 19091, video pushed to our UDP 7078 at the IP we announce).
On macOS and Windows, Docker runs containers in a Linux VM: no Bluetooth, and the IP the app
would announce is the VM's, which the drone cannot reach. So on those systems:
- run the web app natively (`drone/fly`; on Windows see the commands in `drone/patrol/winfly`), and
- run the patrol in Docker against it: profile `patrol` with `SDK_URL=http://host.docker.internal:8077`.

On a Linux host the `drone` service uses the host network (Wi-Fi, UDP) and the host's BlueZ over
D-Bus (`/var/run/dbus`); join the drone's Wi-Fi on the host first.

The `patrol` service starts with `--dry-run` (prints decisions, sends no sticks). Remove it in
`compose.yaml` only after the front distance was checked against tape marks (T1 in #32).
