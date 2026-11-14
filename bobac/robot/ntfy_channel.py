"""Talk to the parents through ntfy push notifications: no account, no phone number, no SMS.

Same interface as caregiver.Telegram, so care.py works with either. Standard library only (runs
on the robot, Python 3.8). Parents install the ntfy app (or open https://ntfy.sh/<topic> in a
browser) and subscribe to NTFY_TOPIC. Configuration comes from the environment:

    NTFY_TOPIC   secret-ish topic name, long and random (anyone who knows it can read it)
    NTFY_NAME    optional, how the robot names whoever replies (default: "Your family")
    NTFY_URL     optional, default https://ntfy.sh (or a self-hosted ntfy server)

Messages go to NTFY_TOPIC. Notification buttons post their answer to "<NTFY_TOPIC>-replies",
which only the robot listens to, so parents do not get notified of their own taps. Text that a
parent publishes to NTFY_TOPIC from the app is read out by the robot.
"""

import json
import logging
import os
import time
import urllib.error
import urllib.request

DEFAULT_URL = "https://ntfy.sh"
DEFAULT_NAME = "Your family"
REQUEST_TIMEOUT_S = 15
MAX_BUTTONS = 3  # ntfy shows at most 3 actions per notification
PHOTO_TITLE = "Photo from the robot"

log = logging.getLogger("ntfy")


class Ntfy:
    def __init__(self, topic: str, name: str = DEFAULT_NAME, base_url: str = DEFAULT_URL):
        self._base = base_url.rstrip("/")
        self._topic = topic
        self._replies = topic + "-replies"
        self._name = name
        self.chat_ids = [topic]  # one "chat": every parent subscribes to the same topic
        self._since = {}  # topic -> id of the last message handled (or a start time)
        self.skip_backlog()
        self._sent_ids = set()  # our own messages, to tell them from what parents write

    def send(self, chat_id: str, text: str, buttons=(), photo_path=None) -> bool:
        """Push a notification. `buttons` is a list of (label, answer or https URL): a URL opens
        it (e.g. the video call), anything else posts that answer back to the robot."""
        if len(buttons) > MAX_BUTTONS:
            log.warning("ntfy shows %d buttons: dropped %s", MAX_BUTTONS, buttons[MAX_BUTTONS:])
        message = {
            "topic": self._topic,
            "title": text.split("\n")[0][:120],
            "message": text,
            "actions": [self._action(label, data) for label, data in buttons[:MAX_BUTTONS]],
        }
        reply = self._request(self._base, json.dumps(message).encode(), "application/json")
        if reply:
            self._sent_ids.add(reply.get("id"))
        # the photo goes after: its upload is slow and must never hold back the call button
        if reply and photo_path and not self._send_photo(photo_path):
            log.warning("photo not sent")
        return reply is not None

    def skip_backlog(self) -> None:
        """Ignore whatever was posted before now."""
        now = str(int(time.time()))
        self._since = {self._topic: now, self._replies: now}

    def events(self, wait_s: int) -> list:
        """Parent actions since the last call: button taps and text they published. Same shape
        as caregiver.Telegram.events(). Waits wait_s between polls."""
        time.sleep(wait_s)
        events = []
        for topic, kind in ((self._replies, "button"), (self._topic, "text")):
            for message in self._poll(topic):
                if message.get("id") in self._sent_ids or not message.get("message"):
                    continue  # our own notification
                event = {"kind": kind, "chat_id": self._topic, "name": self._name}
                event["data" if kind == "button" else "text"] = message["message"].strip()
                events.append(event)
        return events

    def download(self, file_id: str, destination: str) -> bool:
        return False  # ntfy has no voice notes

    def _action(self, label: str, data: str) -> dict:
        if data.startswith("https://"):
            return {"action": "view", "label": label, "url": data}
        url = f"{self._base}/{self._replies}"
        return {"action": "http", "label": label, "url": url, "method": "POST", "body": data,
                "clear": True}  # fmt: skip

    def _send_photo(self, path: str) -> bool:
        with open(path, "rb") as photo:
            content = photo.read()
        headers = {"Filename": os.path.basename(path), "Title": PHOTO_TITLE}
        url = f"{self._base}/{self._topic}"
        reply = self._request(url, content, "image/jpeg", method="PUT", headers=headers)
        if reply:
            self._sent_ids.add(reply.get("id"))
        return reply is not None

    def _poll(self, topic: str) -> list:
        url = f"{self._base}/{topic}/json?poll=1&since={self._since[topic]}"
        try:
            with urllib.request.urlopen(url, timeout=REQUEST_TIMEOUT_S) as response:
                lines = response.read().decode().splitlines()
        except (urllib.error.URLError, OSError) as error:
            log.error("ntfy poll failed: %s", error)
            return []
        messages = [json.loads(line) for line in lines if line.strip()]
        messages = [m for m in messages if m.get("event") == "message"]
        if messages:
            self._since[topic] = messages[-1]["id"]  # next poll starts after this message
        return messages

    def _request(self, url, body: bytes, content_type: str, method="POST", headers=None):
        all_headers = {"Content-Type": content_type}
        all_headers.update(headers or {})
        request = urllib.request.Request(url, data=body, headers=all_headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=REQUEST_TIMEOUT_S) as response:
                return json.load(response)
        except (urllib.error.URLError, OSError, ValueError) as error:
            log.error("ntfy %s %s failed: %s", method, url, error)
            return None


def from_environment(default_name: str = DEFAULT_NAME):
    """The configured ntfy channel, or None if NTFY_TOPIC is not set."""
    topic = os.environ.get("NTFY_TOPIC", "").strip()
    if not topic:
        return None
    name = os.environ.get("NTFY_NAME", "").strip() or default_name
    return Ntfy(topic, name, os.environ.get("NTFY_URL", DEFAULT_URL))
