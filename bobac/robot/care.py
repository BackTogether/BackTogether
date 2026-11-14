"""What the robot does once it is next to the person who fell.

    ask "are you okay?" -> fine: tell the caregiver and close
                        -> help / no answer: alert the caregiver (photo + buttons), triage
                           questions, then stay: calm phrases, guided breathing, the caregiver's
                           ETA, their text and voice messages, next contact if nobody replies,
                           until the caregiver says they arrived. A report closes the episode.

Plain Python 3.8, no ROS: everything that touches hardware is passed in, so it runs on a laptop.
The HTTP bridge (or care_server.py) owns one CareSession: `run()` in a thread, `answer()` from
the screen buttons, `stop()` to abort, `snapshot()` for /status.
"""

import logging
import math
import os
import tempfile
import threading
import time

ASK_ATTEMPTS = 2
ANSWER_WAIT_S = 20
TRIAGE_WAIT_S = 12
POLL_S = 2  # how long one wait for caregiver events lasts
CALM_EVERY_S = 40
NEXT_CONTACT_AFTER_S = 60  # nobody acknowledged: alert the next caregiver
BREATH_CYCLES = 3
BREATH_IN_S = 4
BREATH_OUT_S = 6
MAX_SPOKEN_MESSAGE_CHARS = 300
TIMELINE_IN_SNAPSHOT = 8

TRIAGE_QUESTIONS = (
    ("hit_head", "Did you hit your head?"),
    ("in_pain", "Are you in pain?"),
    ("can_move_legs", "Can you move your legs?"),
)
ETA_BUTTONS = (
    ("I'm on my way (5 min)", "eta:5"),
    ("I'm on my way (15 min)", "eta:15"),
    ("I'm on my way (30 min)", "eta:30"),
    ("I'm calling the emergency services", "emergency"),
)
ARRIVED_BUTTON = (("I have arrived", "arrived"),)

log = logging.getLogger("care")


class CareSession:
    def __init__(self, profile, zone, fall_time, speech, telegram, take_photo, buzz):
        """`speech`: module with say / say_text / play_file. `telegram`: caregiver.Telegram or
        None. `take_photo(path) -> bool` and `buzz() -> None` reach the robot hardware."""
        self._profile = profile
        self._name = profile["name"].split()[0]
        self._zone = zone
        self._fall_time = fall_time
        self._speech = speech
        self._telegram = telegram
        self._take_photo = take_photo
        self._buzz = buzz

        self._lock = threading.Lock()
        self._stopped = threading.Event()
        self._answered = threading.Event()
        self._state = "talking"
        self._updated_at = time.time()
        self._question = None
        self._options = []
        self._answer = None
        self._first_answer = None
        self._triage = {}
        self._caregiver = None
        self._eta_at = None
        self._breath = None
        self._alerted = []  # chat ids already alerted, in order
        self._last_alert_at = 0.0
        self._timeline = []

    # ---------------------------------------------------------------- called by the bridge

    def run(self) -> None:
        """The whole episode; blocks until it is closed or stop() is called."""
        self._note(f"robot is with {self._name} in the {self._zone}")
        self._speech.say_text(f"Hello {self._name}. I am your care robot. I am here.")
        self._first_answer = self._ask_if_ok()
        if self._stopped.is_set():
            return
        if self._first_answer == "ok":
            self._close_fine()
            return
        self._escalate()
        if self._first_answer == "help":
            self._ask_triage()
        self._stay_until_help_arrives()

    def answer(self, value: str) -> bool:
        """A button was pressed on the screen. False if that answer is not expected now."""
        with self._lock:
            if value not in self._options:
                return False
            self._answer = value
        self._answered.set()
        return True

    def stop(self) -> None:
        self._stopped.set()
        self._answered.set()
        self._set_state("idle")

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "answer": self._first_answer,
                "updated_at": self._updated_at,
                "care": {
                    "zone": self._zone,
                    "fall_time": self._fall_time,
                    "question": self._question,
                    "options": list(self._options),
                    "triage": dict(self._triage),
                    "caregiver": self._caregiver,
                    "eta_at": self._eta_at,
                    "alerts_sent": len(self._alerted),
                    "breath": self._breath,
                    "timeline": self._timeline[-TIMELINE_IN_SNAPSHOT:],
                    "profile": self._profile,
                },
            }

    # ---------------------------------------------------------------- steps

    def _ask_if_ok(self):
        for _ in range(ASK_ATTEMPTS):
            self._set_state("talking")
            self._speech.say("are_you_ok")
            self._speech.say("press_button")
            self._set_state("waiting_answer")
            answer = self._wait_answer("Are you okay?", ("ok", "help"), ANSWER_WAIT_S)
            if answer or self._stopped.is_set():
                self._note(f"answered: {answer}")
                return answer
        self._note(f"no answer after {ASK_ATTEMPTS} attempts")
        return None

    def _close_fine(self) -> None:
        self._set_state("talking")
        self._speech.say("all_good")
        self._note("says they are fine: closed")
        self._tell_all_contacts(
            f"ℹ️ {self._name} fell in the {self._zone} at {_clock(self._fall_time)} "
            f"but says they are fine. Please check on them.\n\n{self._report()}"
        )
        self._set_state("idle")

    def _escalate(self) -> None:
        self._set_state("escalated")
        self._speech.say("calling_help" if self._first_answer == "help" else "no_answer")
        self._buzz()
        self._alert_next_contact()
        self._speech.say("do_not_get_up")

    def _ask_triage(self) -> None:
        for key, question in TRIAGE_QUESTIONS:
            if self._stopped.is_set():
                return
            self._speech.say_text(question + " Press yes or no.")
            answer = self._wait_answer(question, ("yes", "no"), TRIAGE_WAIT_S)
            with self._lock:
                self._triage[key] = answer or "no answer"
            self._note("{} -> {}".format(question, answer or "no answer"))
        lines = [f"{q} {self._triage[k]}" for k, q in TRIAGE_QUESTIONS]
        self._tell_alerted("📝 {} answered:\n{}".format(self._name, "\n".join(lines)))

    def _stay_until_help_arrives(self) -> None:
        next_calm_at = time.time() + CALM_EVERY_S
        breathe_next = False
        while not self._stopped.is_set():
            if self._telegram:
                events = self._telegram.events(POLL_S)
            else:
                self._stopped.wait(POLL_S)
                events = []
            for event in events:
                if self._handle_caregiver(event) == "arrived":
                    self._close_arrived()
                    return
            now = time.time()
            nobody_replied = self._caregiver is None
            if nobody_replied and now - self._last_alert_at > NEXT_CONTACT_AFTER_S:
                self._alert_next_contact()
            if now >= next_calm_at:
                if breathe_next:
                    self._guide_breathing()
                else:
                    self._reassure()
                breathe_next = not breathe_next
                next_calm_at = time.time() + CALM_EVERY_S

    def _close_arrived(self) -> None:
        self._speech.say("help_arrived")
        self._note(f"{self._caregiver} arrived: closed")
        self._tell_alerted("✅ Episode closed.\n\n" + self._report())
        self._set_state("idle")

    # ---------------------------------------------------------------- caregiver

    def _alert_next_contact(self) -> None:
        self._last_alert_at = time.time()
        if not self._telegram:
            self._note("no caregiver channel configured: nobody was alerted")
            return
        pending = [c for c in self._telegram.chat_ids if c not in self._alerted]
        if not pending:
            return
        chat_id = pending[0]
        answer = {"help": "asked for help", None: "did NOT answer"}[self._first_answer]
        text = (
            f"🚨 {self._name} fell in the {self._zone} at {_clock(self._fall_time)} "
            f"and {answer}. The robot is with them."
        )
        photo_path = os.path.join(tempfile.gettempdir(), "bobac_care_photo.jpg")
        has_photo = self._take_photo(photo_path)
        sent = self._telegram.send(
            chat_id, text, buttons=ETA_BUTTONS, photo_path=photo_path if has_photo else None
        )
        self._alerted.append(chat_id)
        self._note(
            "alert {} to contact {} ({})".format(
                "sent" if sent else "FAILED",
                len(self._alerted),
                "with photo" if has_photo else "no photo",
            )
        )

    def _handle_caregiver(self, event: dict):
        name = event["name"]
        if event["kind"] == "button" and event["data"].startswith("eta:"):
            minutes = int(event["data"].split(":")[1])
            with self._lock:
                self._caregiver = name
                self._eta_at = time.time() + minutes * 60
            self._note(f"{name} is on the way, about {minutes} min")
            self._speech.say_text(
                f"{self._name}, {name} is coming and will be here in about {minutes} minutes."
            )
            self._telegram.send(
                event["chat_id"],
                f"I told {self._name}. Text or voice messages you send here are played by "
                "the robot. Press the button when you are there.",
                buttons=ARRIVED_BUTTON,
            )
        elif event["kind"] == "button" and event["data"] == "emergency":
            with self._lock:
                self._caregiver = name
            self._note(f"{name} is calling the emergency services")
            self._speech.say_text(f"{name} is calling the emergency services.")
            self._telegram.send(event["chat_id"], f"I told {self._name}.", buttons=ARRIVED_BUTTON)
        elif event["kind"] == "button" and event["data"] == "arrived":
            with self._lock:
                self._caregiver = name
            return "arrived"
        elif event["kind"] == "text":
            self._note("message from {}: {}".format(name, event["text"]))
            spoken = event["text"][:MAX_SPOKEN_MESSAGE_CHARS]
            self._speech.say_text(f"Message from {name}: {spoken}")
        elif event["kind"] == "voice":
            self._note(f"voice message from {name}")
            voice_path = os.path.join(tempfile.gettempdir(), "bobac_care_voice.oga")
            if self._telegram.download(event["file_id"], voice_path):
                self._speech.say_text(f"Voice message from {name}.")
                self._speech.play_file(voice_path)
        return None

    def _tell_alerted(self, text: str) -> None:
        if self._telegram:
            for chat_id in self._alerted:
                self._telegram.send(chat_id, text)

    def _tell_all_contacts(self, text: str) -> None:
        if self._telegram:
            for chat_id in self._telegram.chat_ids:
                self._telegram.send(chat_id, text)

    # ---------------------------------------------------------------- calming

    def _reassure(self) -> None:
        if self._eta_at is None:
            self._speech.say("still_here" if self._caregiver is None else "help_on_way")
            return
        minutes_left = max(1, math.ceil((self._eta_at - time.time()) / 60))
        self._speech.say_text(
            "{} will be here in about {} {}. I am staying with you.".format(
                self._caregiver, minutes_left, "minute" if minutes_left == 1 else "minutes"
            )
        )

    def _guide_breathing(self) -> None:
        self._speech.say_text(f"Let us breathe together, {self._name}.")
        for _ in range(BREATH_CYCLES):
            for phase, phrase, seconds in (
                ("in", "breathe_in", BREATH_IN_S),
                ("out", "breathe_out", BREATH_OUT_S),
            ):
                self._set_breath(phase)
                started = time.time()
                self._speech.say(phrase)
                remaining = seconds - (time.time() - started)
                if self._stopped.wait(max(0.0, remaining)):
                    self._set_breath(None)
                    return
        self._set_breath(None)

    # ---------------------------------------------------------------- helpers

    def _wait_answer(self, question: str, options, timeout_s: float):
        with self._lock:
            self._question = question
            self._options = list(options)
            self._answer = None
        self._answered.clear()
        self._answered.wait(timeout_s)
        with self._lock:
            self._question = None
            self._options = []
            return self._answer

    def _set_state(self, state: str) -> None:
        if self._stopped.is_set() and state != "idle":
            return  # a step that was already running must not undo stop()
        with self._lock:
            self._state = state
            self._updated_at = time.time()

    def _set_breath(self, phase) -> None:
        with self._lock:
            self._breath = phase

    def _note(self, event: str) -> None:
        log.info(event)
        with self._lock:
            self._timeline.append({"time": time.time(), "event": event})

    def _report(self) -> str:
        with self._lock:
            lines = ["{}  {}".format(_clock(e["time"]), e["event"]) for e in self._timeline]
        minutes = (time.time() - self._fall_time) / 60
        header = (
            f"Report: fall at {_clock(self._fall_time)} in the {self._zone}, {minutes:.0f} min ago."
        )
        return "\n".join([header] + lines)


def _clock(epoch_s: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(epoch_s))
