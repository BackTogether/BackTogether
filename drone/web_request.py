"""Wait for a parent's search request from the web page and make it the child to look for.

The page (web/) sends the child's name and photos to an ntfy topic; scripts/web_inbox.py (the
same code drone/inbox runs) saves them in drone/identify/people/<name>/. A request whose photos
show no face big enough to recognise (people.py ignores reference faces under
MIN_REFERENCE_FACE_PX: a full-body photo shrunk by the page) is reported and the wait goes on,
so the parent can send close-ups instead.
"""

import importlib.util
import logging
from pathlib import Path

import cv2

from drone.identify.faces import FaceEmbedder
from drone.identify.people import MIN_REFERENCE_FACE_PX, PEOPLE_DIR

WEB_INBOX = Path(__file__).resolve().parent.parent / "scripts" / "web_inbox.py"

log = logging.getLogger("find_child")


def _web_inbox():
    """scripts/web_inbox.py as a module (scripts/ is not a package)."""
    spec = importlib.util.spec_from_file_location("web_inbox", WEB_INBOX)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def usable_photos(folder: Path, embedder: FaceEmbedder) -> int:
    """The web photos in `folder` whose largest face people.py would keep as a reference."""
    count = 0
    for photo in sorted(folder.glob("photo-*.jpg")):
        image = cv2.imread(str(photo))
        faces = embedder.faces(image) if image is not None else []
        if faces and min(faces[0].box[2:]) >= MIN_REFERENCE_FACE_PX:
            count += 1
    return count


def wait_for_child() -> str:
    """Block until a request with at least one usable face arrives; the child's folder name."""
    inbox = _web_inbox()
    embedder = FaceEmbedder()
    topic = inbox.topic_url_from_config()
    log.info("waiting for the parent's request from the web page (%s)", topic)
    for event in inbox.listen(topic):
        request = inbox.read_request(event)
        if request is None:
            continue
        folder, saved = inbox.save(request, PEOPLE_DIR)
        usable = usable_photos(folder, embedder) if saved else 0
        name = (request.get("child") or {}).get("name") or folder.name
        if usable:
            log.info("request for %s: %d photo(s), %d with a usable face", name, saved, usable)
            return folder.name
        log.warning(
            "request for %s: %d photo(s) but no face of %d px or more: waiting for another "
            "request (ask for close-ups of the face)",
            name,
            saved,
            MIN_REFERENCE_FACE_PX,
        )
    raise RuntimeError("the web request stream ended")
