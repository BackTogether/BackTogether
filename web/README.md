# web/ — "Find My Child" site

The public page parents use during an event: they describe their child, add a photo, and the page
POSTs a search request that starts the drone. While the request goes out, a 3D view shows the
Hover Air X1 lifting off from just above the Bobac3 base and flying a search grid.

Deployed to GitHub Pages by [.github/workflows/pages.yml](../.github/workflows/pages.yml) on every
push to `main` that touches `web/`.

## Files
| File | What it does |
|---|---|
| `index.html` | the whole page: hero, "how it works", request form, search panel |
| `styles.css` | dark control-room theme, sticky 3D column, mobile layout |
| `config.js` | **the only file you edit to point the site at the robot** (endpoint, token, zones) |
| `app.js` | form, photo downscaling, the POST, and the search view driven by the 3D stage |
| `scene.js` | three.js stage: ground, crowd, pad, the Bobac3 with its screen, the X1, and the choreography |
| `vendor/three.module.min.js` | three.js r160, vendored so the demo works without wifi |
| `favicon.svg` | tab icon |

## Run it locally
Any static file server works (ES modules do not load from `file://`):

```bash
cd web && python3 -m http.server 8080     # then open http://localhost:8080
```

Check it before pushing (this is what CI runs):

```bash
python3 scripts/check_web.py              # every referenced file exists
```

## Point it at the robot
Edit [config.js](config.js):

```js
window.FINDMYCHILD_CONFIG = {
  apiUrl: "https://robot.example.org/api/v1/search-requests",
  apiToken: "",            // optional, sent as "Authorization: Bearer <token>"
  eventName: "Leukerbad Open Air",
  zones: ["Main stage", "Food court", "…"],
};
```

With `apiUrl` empty the page runs in **demo mode**: the full animation plays, nothing is sent, and
the search panel says so. That is the state it ships in, so the deployed page is safe to show.

Two things the endpoint must do, or the browser blocks the request:
- answer the `OPTIONS` preflight and send `Access-Control-Allow-Origin` for the Pages origin
  (`https://gra1111.github.io`) plus `Access-Control-Allow-Headers: Content-Type, Authorization`;
- be **https** if the site is served over https. A plain-http robot on the event LAN cannot be
  called from the deployed page: either put a TLS proxy in front of it, or serve this folder from
  a laptop on the same network during the demo.

Anything in `config.js` is public once deployed. Use a token that only starts a search.

## What the drone side receives
`POST <apiUrl>`, `Content-Type: application/json`:

```json
{
  "schema": "find-my-child/search-request@1",
  "request_id": "FMC-20261116-9F3A2B10",
  "requested_at": "2026-11-16T13:41:07.182Z",
  "event": "Leukerbad Open Air",
  "parent": { "name": "Anna Weber", "phone": "+41 79 123 45 67" },
  "child": { "name": "Mia", "age": 6, "description": "Red rain jacket, blond ponytail" },
  "last_seen": { "zone": "Food court", "minutes_ago": 12, "note": "Walked towards the stands" },
  "photo": {
    "filename": "mia.jpg",
    "media_type": "image/jpeg",
    "width": 768,
    "height": 1024,
    "data": "/9j/4AAQSkZJRgABAQ…"
  },
  "client": { "page": "https://…", "user_agent": "…" }
}
```

- `photo.data` is base64 **without** the `data:` prefix: `base64.b64decode(data)` gives a JPEG.
  The browser downscales to 1024 px on the longest side first, so a request is around 150 kB.
- `request_id` is generated in the browser and shown to the parent; use it to tie the search,
  the photo and any result together.
- Anything `2xx` counts as accepted. The page shows "Request sent to the robot" and starts the
  launch animation; any other status, a timeout (12 s) or a CORS failure shows the error to the
  parent and does not pretend the drone is flying.

A minimal receiver, enough to plug the real drone in later:

```python
from http.server import BaseHTTPRequestHandler, HTTPServer
import base64, json, pathlib

class Handler(BaseHTTPRequestHandler):
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")

    def do_OPTIONS(self):
        self.send_response(204); self._cors(); self.end_headers()

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        photo = base64.b64decode(request["photo"]["data"])
        pathlib.Path(f"{request['request_id']}.jpg").write_bytes(photo)
        print("search for", request["child"]["name"], "in", request["last_seen"]["zone"])
        # TODO: hand request_id + the photo to the drone
        self.send_response(202); self._cors()
        self.send_header("Content-Type", "application/json"); self.end_headers()
        self.wfile.write(b'{"accepted": true}')

HTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
```

## The 3D stage
`scene.js` builds everything from primitives, so there is no model file to load. Phases:
`idle` → `launch` (request received, spin up, hop and lift off) → `search` (lawnmower
grid with the scan cone and trail) → `found` (the drone hovers over the child, the Bobac3 turns
and drives there) → `recall` (the Bobac3 drives home with the drone following, then it settles over the deck) →
`idle`. `recall` also works straight from `launch` or `search`. `app.js` drives the phases and
reads `stage.telemetry()` for the altitude / speed / battery / coverage readouts.

- **`found` is automatic in demo mode only**, at `DEMO_FOUND_COVERAGE` (20 %) in `app.js`. With a
  real endpoint nothing reports a sighting yet: call `stage.setPhase("found")` when it does.
- Every speed, rate and damping is in the `MOTION` block of `scene.js`; step durations are in
  the step tables right under it.

- Drag to orbit, scroll to zoom. The scripted framing eases back in a few seconds after you let go.
- `prefers-reduced-motion` shortens every step and stops the auto-orbit.
- Without WebGL the canvas is replaced by a short message and the form still works.
- The Bobac3 follows the real robot: 377 mm across, three omni wheels, and the lidar, depth
  camera, ultrasonic and head camera positions from `bobac3_description/xacro`. The vendor meshes
  are not in the repo, so the shapes in between are approximations, and the screen position on
  the tower is placed by eye (`BOBAC.screenY`, `screenZ`, `screenTilt`), with the head camera
  clipped to its top edge. No arm: ours has none.
- It drives and turns at the real navigation caps (0.4 m/s, 0.8 rad/s).
- The X1 uses its published size (127 × 145 × 30 mm unfolded). While it waits it hovers a few
  centimetres over the deck, rotors turning; it is drawn 1.7× bigger in the air (`MOTION.drone.airScale`), or it would be a dot at search altitude.

## Deployment
- Push to `main` with changes under `web/` → the workflow checks the site, then deploys it.
- The repository needs **Settings → Pages → Source: GitHub Actions** once, or the deploy step
  fails with "Pages site not found".
- Published at `https://gra1111.github.io/leukerbard-robotics-AI-hackathon-2026/`.
- `workflow_dispatch` in the Actions tab deploys a chosen branch, which is how to put a demo
  version up without merging to `main`.
