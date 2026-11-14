"""Talk to the parents through a Telegram bot: alerts with photo and buttons, and their replies.

Standard library only (runs on the robot, Python 3.8). Configuration comes from the environment,
never from the repo:

    TELEGRAM_BOT_TOKEN   token from @BotFather
    TELEGRAM_CHAT_IDS    comma-separated chat ids, one per parent
    TELEGRAM_API_URL     optional, default https://api.telegram.org (override to test locally)
"""

import json
import logging
import os
import urllib.error
import urllib.request
import uuid

DEFAULT_API_URL = "https://api.telegram.org"
REQUEST_TIMEOUT_S = 15
MULTIPART_BOUNDARY_PREFIX = "----bobac"

log = logging.getLogger("caregiver")


class Telegram:
    def __init__(self, token: str, chat_ids, api_url: str = DEFAULT_API_URL):
        self._base = "{}/bot{}".format(api_url.rstrip("/"), token)
        self._file_base = "{}/file/bot{}".format(api_url.rstrip("/"), token)
        self.chat_ids = list(chat_ids)
        self._next_update_id = None

    def send(self, chat_id: str, text: str, buttons=(), photo_path=None) -> bool:
        """Send a message; `buttons` is a list of (label, callback_data or https URL), one per
        row. A URL becomes a link button (e.g. a video call), anything else a reply button."""
        fields = {"chat_id": chat_id}
        if buttons:
            rows = [[_button(label, data)] for label, data in buttons]
            fields["reply_markup"] = json.dumps({"inline_keyboard": rows})
        if photo_path:
            fields["caption"] = text
            with open(photo_path, "rb") as photo:
                reply = self._call("sendPhoto", fields, photo=photo.read())
        else:
            fields["text"] = text
            reply = self._call("sendMessage", fields)
        return reply is not None

    def skip_backlog(self) -> None:
        """Ignore whatever was sent to the bot before now."""
        reply = self._call("getUpdates", {"offset": -1, "timeout": 0})
        if reply:
            self._next_update_id = reply[-1]["update_id"] + 1

    def events(self, wait_s: int) -> list:
        """Wait up to wait_s for parent actions. Each event is a dict with `kind`
        (`button`, `text` or `voice`), `chat_id`, `name` and `data` / `text` / `file_id`."""
        fields = {"timeout": wait_s}
        if self._next_update_id is not None:
            fields["offset"] = self._next_update_id
        updates = self._call("getUpdates", fields, timeout_s=wait_s + REQUEST_TIMEOUT_S) or []
        events = []
        for update in updates:
            self._next_update_id = update["update_id"] + 1
            event = self._to_event(update)
            if event and event["chat_id"] in self.chat_ids:
                events.append(event)
            elif event:
                log.warning("ignored message from unknown chat %s", event["chat_id"])
        return events

    def download(self, file_id: str, destination: str) -> bool:
        info = self._call("getFile", {"file_id": file_id})
        if not info:
            return False
        url = "{}/{}".format(self._file_base, info["file_path"])
        try:
            with urllib.request.urlopen(url, timeout=REQUEST_TIMEOUT_S) as response:
                content = response.read()
        except (urllib.error.URLError, OSError) as error:
            log.error("download failed: %s", error)
            return False
        with open(destination, "wb") as output:
            output.write(content)
        return True

    def _to_event(self, update: dict):
        callback = update.get("callback_query")
        if callback:
            self._call("answerCallbackQuery", {"callback_query_id": callback["id"]})
            chat_id = str(callback["message"]["chat"]["id"])
            name = callback["from"].get("first_name", "Your family")
            return {"kind": "button", "chat_id": chat_id, "name": name, "data": callback["data"]}
        message = update.get("message")
        if not message:
            return None
        event = {
            "chat_id": str(message["chat"]["id"]),
            "name": message.get("from", {}).get("first_name", "Your family"),
        }
        voice = message.get("voice") or message.get("audio")
        if voice:
            event.update(kind="voice", file_id=voice["file_id"])
        elif message.get("text"):
            event.update(kind="text", text=message["text"])
        else:
            return None
        return event

    def _call(self, method: str, fields: dict, photo=None, timeout_s: float = REQUEST_TIMEOUT_S):
        """POST to the Bot API; returns the `result` value, or None on any failure."""
        url = f"{self._base}/{method}"
        if photo is None:
            body = json.dumps(fields).encode()
            content_type = "application/json"
        else:
            body, content_type = _multipart(fields, photo)
        request = urllib.request.Request(url, data=body, headers={"Content-Type": content_type})
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                reply = json.load(response)
        except (urllib.error.URLError, OSError, ValueError) as error:
            log.error("telegram %s failed: %s", method, error)
            return None
        if not reply.get("ok"):
            log.error("telegram %s refused: %s", method, reply.get("description"))
            return None
        return reply["result"]


def _button(label: str, data: str) -> dict:
    if data.startswith("https://"):
        return {"text": label, "url": data}
    return {"text": label, "callback_data": data}


def _multipart(fields: dict, photo: bytes):
    boundary = MULTIPART_BOUNDARY_PREFIX + uuid.uuid4().hex
    parts = []
    for name, value in fields.items():
        header = f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n'
        parts.append(f"{header}{value}\r\n".encode())
    photo_header = (
        f"--{boundary}\r\n"
        'Content-Disposition: form-data; name="photo"; filename="photo.jpg"\r\n'
        "Content-Type: image/jpeg\r\n\r\n"
    )
    parts.append(photo_header.encode())
    parts.append(photo)
    parts.append(f"\r\n--{boundary}--\r\n".encode())
    return b"".join(parts), "multipart/form-data; boundary=" + boundary


def from_environment():
    """The configured Telegram channel, or None if the bot token or chat ids are not set."""
    token = os.environ.get("TELEGRAM_BOT_TOKEN", "")
    chat_ids = [c.strip() for c in os.environ.get("TELEGRAM_CHAT_IDS", "").split(",") if c.strip()]
    if not token or not chat_ids:
        log.warning("TELEGRAM_BOT_TOKEN / TELEGRAM_CHAT_IDS not set: parents will not be told")
        return None
    return Telegram(token, chat_ids, os.environ.get("TELEGRAM_API_URL", DEFAULT_API_URL))
