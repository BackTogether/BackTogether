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

## Where the request goes: an ntfy topic
`apiUrl` in [config.js](config.js) is an [ntfy](https://ntfy.sh) topic: it accepts posts from this
page (it sends `Access-Control-Allow-Origin: *`) and keeps a large body, like a request with
photos, as an attachment for a few hours. Nothing on the robot or the drone laptop has to be
reachable from the parent's phone. The receiver, on the robot or the drone laptop (standard
library, Python 3.8+), saves each request in a folder named after the child in capitals:

```bash
python3 scripts/web_inbox.py                              # ~/search_requests/JORGE/photo-1.jpg ...
drone/inbox                                               # drone laptop: drone/identify/people/ (Windows: drone\inbox)
```
On the robot it runs as a service from boot, into `~/.reinovo/face_data/` where face_rec reads
faces (`bobac/vendor/bobac3_ws/src/bringup/systemd/web-inbox.service`, install steps inside).
Then search with the name it printed: `drone/find JORGE` on the drone laptop, and the robot's
demo launched (or relaunched: face_rec loads faces when it starts) with `person:=jorge`.

It reads the topic from this same `apiUrl` line. Only requests sent while it runs are saved; a new
request for the same name replaces that folder's photos. With `apiUrl` empty nothing is sent.

Anything in `config.js` is public once deployed, the topic included: anyone can post to it and
read what is posted. Send only test photos of the team, never of a real child.

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
  "photos": [ { "filename": "mia.jpg", "…": "…", "data": "/9j/…" } ],
  "client": { "page": "https://…", "user_agent": "…" }
}
```

- Only `child.name` and the photos are ever sent: the form has no other fields, so the drone side
  must not expect a parent, an age, a description or a zone. Schema bumped to `@2` for that.
- `photos` holds every photo the parent picked (up to `photoMaxCount`, 5); `photo` is the first,
  for older readers. Each `data` is base64 **without** the `data:` prefix:
  `base64.b64decode(data)` gives a JPEG. The browser downscales each to 1024 px on the longest side
  first, about 150 kB per photo.
- `request_id` is generated in the browser and shown to the parent; use it to tie the search,
  the photo and any result together.
- Anything `2xx` counts as accepted. The page shows "Request sent to the robot" and starts the
  launch animation; any other status, a timeout (12 s) or a CORS failure shows the error to the
  parent and does not pretend the drone is flying.

The receiver is [scripts/web_inbox.py](../scripts/web_inbox.py) (see above).

## Deployment
- Push to `main` with changes under `web/` → the workflow checks the site, then deploys it.
- The repository needs **Settings → Pages → Source: GitHub Actions** once, or the deploy step
  fails with "Pages site not found".
- Published at `https://gra1111.github.io/leukerbard-robotics-AI-hackathon-2026/`.
- `workflow_dispatch` in the Actions tab deploys a chosen branch, which is how to put a demo
  version up without merging to `main`.
