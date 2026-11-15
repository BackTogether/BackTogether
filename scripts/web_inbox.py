"""Receive the parents' search requests from the web page and save the child's photos.

The page (web/) POSTs each request to an ntfy topic (`apiUrl` in web/config.js). This listens to
that topic and saves every request in a folder named after the child, in lower case, so
"JoRgE", "JORGE" and "jorge" all land in the same folder:

    <out>/jorge/photo-1.jpg, photo-2.jpg ...   the photos the parent sent: nothing else goes in
                                               there (the robot's face_rec opens every file of the
                                               folder as a photo of a face)
    ~/web_requests/jorge.json                  name, request id and time (no photo bytes): what
                                               kid_care reads to greet the child by name

A later request for the same name replaces the photos it saved there (photo-*.jpg); photos added
by hand to that folder stay. Only requests sent while this runs are saved. Standard library only,
Python 3.8+: the same file runs on the robot and on the drone laptop.

    python3 scripts/web_inbox.py                              # saves in ~/search_requests
    python3 scripts/web_inbox.py --out ~/.reinovo/face_data --check-faces   # the robot (service)
    python3 scripts/web_inbox.py --out drone/identify/people  # the drone laptop (drone/inbox)

--check-faces keeps only the photos where face_recognition (installed on the robot) finds a face:
face_rec takes the first face of every photo and stops on one without any.
"""

import argparse
import base64
import binascii
import json
import logging
import re
import time
import unicodedata
import urllib.request
from pathlib import Path

WEB_CONFIG = Path(__file__).resolve().parent.parent / "web" / "config.js"
DEFAULT_OUT = Path.home() / "search_requests"
DEFAULT_REQUESTS = Path.home() / "web_requests"  # one <name>.json per child, out of the photos
SCHEMA_PREFIX = "find-my-child/search-request@"
JPEG_START = b"\xff\xd8"
STREAM_TIMEOUT_S = 120.0
DOWNLOAD_TIMEOUT_S = 30.0
RECONNECT_S = 3.0

log = logging.getLogger("web_inbox")
# No proxy, whatever the system settings say (the robot's environment sets none it needs)
_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def topic_url_from_config(config=WEB_CONFIG):
    """The page's `apiUrl`: the ntfy topic it posts to."""
    match = re.search(r'apiUrl:\s*"([^"]*)"', config.read_text(encoding="utf-8"))
    url = match.group(1) if match else ""
    if "ntfy" not in url:
        raise SystemExit(f"{config}: apiUrl is not an ntfy topic ({url or 'empty'})")
    return url


def folder_name(child_name, fallback):
    """'Jorge Martín' -> 'jorge_martin': lower case, letters, digits and underscores only."""
    ascii_name = unicodedata.normalize("NFKD", child_name).encode("ascii", "ignore").decode()
    return re.sub(r"[^a-z0-9]+", "_", ascii_name.lower()).strip("_") or fallback.lower()


def read_request(event):
    """The page's request from one ntfy event (a large body arrives as an attachment), or None."""
    try:
        attachment = event.get("attachment")
        if attachment:
            with _opener.open(attachment["url"], timeout=DOWNLOAD_TIMEOUT_S) as response:
                text = response.read().decode("utf-8")
        else:
            text = event.get("message", "")
        request = json.loads(text)
    except (OSError, ValueError, KeyError) as error:
        log.warning("skipped a message that is not a search request: %s", error)
        return None
    schema = str(request.get("schema", "")) if isinstance(request, dict) else ""
    if not schema.startswith(SCHEMA_PREFIX):
        log.warning("skipped a message that is not a search request")
        return None
    return request


def has_face(data):
    """True if face_recognition finds a face in the JPEG `data` (what face_rec will need)."""
    import io

    import face_recognition  # only with --check-faces: the robot has it, the laptops need not

    return bool(face_recognition.face_locations(face_recognition.load_image_file(io.BytesIO(data))))


def save(request, out, requests_dir, check_faces=False):
    """Write the request's photos in out/<name>/ and its summary in requests_dir/<name>.json.
    Returns (folder, photos saved)."""
    request_id = str(request.get("request_id") or "request")
    name = str((request.get("child") or {}).get("name") or "").strip()
    folder = out / folder_name(name, request_id)
    photos = request.get("photos") or [request.get("photo") or {}]
    images = []
    for index, photo in enumerate(photos, start=1):
        try:
            data = base64.b64decode(photo.get("data") or "", validate=True)
        except (binascii.Error, AttributeError, ValueError):
            data = b""
        if not data.startswith(JPEG_START):
            log.warning("photo %d of %s is not a JPEG: skipped", index, request_id)
        elif check_faces and not has_face(data):
            log.warning("photo %d of %s has no face it can find: skipped", index, request_id)
        else:
            images.append(data)
    if not images:
        return folder, 0
    folder.mkdir(parents=True, exist_ok=True)
    for old in folder.glob("photo-*.jpg"):  # a new request for this name replaces its photos
        old.unlink()
    for index, data in enumerate(images, start=1):
        (folder / f"photo-{index}.jpg").write_bytes(data)
    summary = {
        "name": name,
        "request_id": request_id,
        "requested_at": request.get("requested_at"),
        "photos": len(images),
    }
    text = json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    requests_dir.mkdir(parents=True, exist_ok=True)
    (requests_dir / f"{folder.name}.json").write_text(text, encoding="utf-8")
    return folder, len(images)


def listen(topic_url):
    """Yield every new ntfy message on the topic, reconnecting when the link drops."""
    since = str(int(time.time()))  # only requests sent from now on
    while True:
        try:
            url = f"{topic_url}/json?since={since}"
            with _opener.open(url, timeout=STREAM_TIMEOUT_S) as stream:
                log.info("listening on %s", topic_url)
                for line in stream:
                    event = json.loads(line)
                    if event.get("event") == "message":
                        since = event["id"]  # after a reconnect, carry on from here
                        yield event
        except (OSError, ValueError) as error:
            log.warning("ntfy link lost (%s), reconnecting", error)
        time.sleep(RECONNECT_S)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="where the folders go")
    parser.add_argument("--topic-url", help="ntfy topic (default: apiUrl in web/config.js)")
    parser.add_argument(
        "--requests", type=Path, default=DEFAULT_REQUESTS, help="where <name>.json goes"
    )
    parser.add_argument(
        "--check-faces", action="store_true", help="keep only photos with a face (needs it)"
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    out = args.out.expanduser()
    for event in listen(args.topic_url or topic_url_from_config()):
        request = read_request(event)
        if request is None:
            continue
        folder, saved = save(request, out, args.requests.expanduser(), args.check_faces)
        if saved:
            log.info("%s: %d photo(s) saved in %s", request.get("request_id"), saved, folder)
        else:
            log.warning("%s: no usable photo", request.get("request_id"))


if __name__ == "__main__":
    main()
