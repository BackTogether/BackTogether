# web/ — "Find My Child" site

The public page parents use during an event. It asks for two things only — the child's **name**
and a **photo** — and POSTs a search request that starts the drone. Once it is sent, the page
only says that the drone is on its way and that a notification with the video call follows. It is
built for a phone held in one hand: one simple column.

Deployed to GitHub Pages by [.github/workflows/pages.yml](../.github/workflows/pages.yml) on every
push to `main` that touches `web/`.

## Files
| File | What it does |
|---|---|
| `index.html` | the whole page: hero, request form (name + photo), confirmation, "how it works" |
| `styles.css` | dark control-room theme, mobile layout |
| `config.js` | **the only file you edit to point the site at the robot** (endpoint, token, event name) |
| `app.js` | form, photo downscaling, the POST, and the confirmation |
| `favicon.svg` | tab icon |
| `pitch.html`, `pitch-poster.jpg` | the one slide left on the screen while we pitch (`F` = full screen), with the Leukerbad Robotics Carnival '26 poster (the organisers' image, from the event page) |

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
  "schema": "find-my-child/search-request@2",
  "request_id": "FMC-20261116-9F3A2B10",
  "requested_at": "2026-11-16T13:41:07.182Z",
  "event": "Leukerbad Open Air",
  "child": { "name": "Mia" },
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

- Only `child.name` and `photo` are ever sent: the form has no other fields, so the drone side
  must not expect a parent, an age, a description or a zone. Schema bumped to `@2` for that.
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
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        photo = base64.b64decode(request["photo"]["data"])
        pathlib.Path(f"{request['request_id']}.jpg").write_bytes(photo)
        print("search for", request["child"]["name"])
        # TODO: hand request_id + the photo to the drone
        self.send_response(202)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"accepted": true}')


HTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
```

## Deployment
- Push to `main` with changes under `web/` → the workflow checks the site, then deploys it.
- The repository needs **Settings → Pages → Source: GitHub Actions** once, or the deploy step
  fails with "Pages site not found".
- Published at `https://gra1111.github.io/leukerbard-robotics-AI-hackathon-2026/`.
- `workflow_dispatch` in the Actions tab deploys a chosen branch, which is how to put a demo
  version up without merging to `main`.
