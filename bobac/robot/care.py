"""What the robot does once it reaches the lost child the drone found.

    greet the child by name -> "are you okay?" -> "did you get hurt?"
    -> tell the parents at once (photo, answers, video call link, "on my way" buttons)
    -> video call with the parent on the robot's screen
    -> keep the child company (calm phrases, a breathing game, the parent's ETA, their text and
       voice messages) until a parent says they are with the child. A report closes it.

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
import uuid

ASK_ATTEMPTS = 2
ANSWER_WAIT_S = 20
HURT_WAIT_S = 15
POLL_S = 2  # how long one wait for parent events lasts
CALM_EVERY_S = 40
BREATH_CYCLES = 3
BREATH_IN_S = 4
BREATH_OUT_S = 6
MAX_SPOKEN_MESSAGE_CHARS = 300
TIMELINE_IN_SNAPSHOT = 8
# Public Jitsi server that lets anyone open a room without logging in (meet.jit.si no longer
# does). The robot and the parent both open the same room URL.
DEFAULT_CALL_BASE_URL = "https://meet.ffmuc.net"
CALL_URL_OPTIONS = (
    "#config.prejoinConfig.enabled=false&config.startWithAudioMuted=false"
    "&config.startWithVideoMuted=false"
)

ETA_BUTTONS = (
    ("I'm on my way (5 min)", "eta:5"),
    ("I'm on my way (15 min)", "eta:15"),
    ("I'm on my way (30 min)", "eta:30"),
)
WITH_CHILD_BUTTON = (("I'm with my child now", "arrived"),)
# words that mean each screen answer when said out loud (only the current options are used)
SPOKEN_ANSWERS = {
    "ok": ("yes", "fine", "okay", "good"),
    "help": ("help", "no"),
    "yes": ("yes", "yeah"),
    "no": ("no", "nope"),
}
HOW_ARE_YOU = {"ok": "says they are okay", "help": "says they are NOT okay", None: "did not answer"}
HURT = {"yes": "says they got hurt", "no": "says they are not hurt", None: "did not say if hurt"}

log = logging.getLogger("care")


class CareSession:
    def __init__(
        self,
        child,
        zone,
        found_time,
        speech,
        telegram,
        take_photo,
        buzz,
        listen=None,
        call_base="",
        show_call=None,
    ):
        """`child`: profile dict (config/child.json). `speech`: module with say / say_text /
        play_file. `telegram`: caregiver.Telegram (one chat per parent) or None. `take_photo(path)
        -> bool` and `buzz() -> None` reach the robot hardware. `listen`: module with wait_word
        (listen.py) to also accept spoken answers, or None. `call_base`: Jitsi server URL.
        `show_call(url or None)`: opens / closes the video call on the robot screen, or None."""
        self._child = child
        self._name = child["name"].split()[0]
        self._parent = child["parents"][0]["called"]
        self._zone = zone
        self._found_time = found_time
        self._speech = speech
        self._telegram = telegram
        self._take_photo = take_photo
        self._buzz = buzz
        self._listen = listen
        self._show_call = show_call
        room = f"find-{self._name.lower()}-{uuid.uuid4().hex[:8]}"
        self._call_url = "{}/{}".format((call_base or DEFAULT_CALL_BASE_URL).rstrip("/"), room)

        self._lock = threading.Lock()
        self._stopped = threading.Event()
        self._answered = threading.Event()
        self._state = "talking"
        self._updated_at = time.time()
        self._question = None
        self._options = []
        self._answer = None
        self._how_are_you = None
        self._hurt = None
        self._parent_coming = None  # name of the parent who replied
        self._eta_at = None
        self._call_active = False
        self._breath = None
        self._timeline = []

    # ---------------------------------------------------------------- called by the bridge

    def run(self) -> None:
        """The whole episode; blocks until a parent is with the child or stop() is called."""
        self._note(f"robot is with {self._name} in the {self._zone}")
        self._speech.say_text(
            f"Hi {self._name}! I am a friendly robot. Your {self._parent} sent me to find you."
        )
        self._how_are_you = self._ask_how_are_you()
        if self._stopped.is_set():
            return
        self._hurt = self._ask_if_hurt()
        if self._stopped.is_set():
            return
        self._comfort_after_answers()
        self._tell_parents()
        self._start_call()
        self._stay_until_parent_arrives()

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
        self._end_call()
        self._set_state("idle")

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "answer": self._how_are_you,
                "updated_at": self._updated_at,
                "care": {
                    "zone": self._zone,
                    "found_time": self._found_time,
                    "question": self._question,
                    "options": list(self._options),
                    "how_are_you": self._how_are_you,
                    "hurt": self._hurt,
                    "parent_coming": self._parent_coming,
                    "eta_at": self._eta_at,
                    "call_url": self._call_url if self._call_active else None,
                    "breath": self._breath,
                    "listening": self._listen is not None and bool(self._options),
                    "timeline": self._timeline[-TIMELINE_IN_SNAPSHOT:],
                    "child": self._child,
                },
            }

    # ---------------------------------------------------------------- steps

    def _ask_how_are_you(self):
        for _ in range(ASK_ATTEMPTS):
            self._set_state("talking")
            self._speech.say("are_you_ok")
            self._speech.say("press_button")
            self._set_state("waiting_answer")
            answer = self._wait_answer("Are you okay?", ("ok", "help"), ANSWER_WAIT_S)
            if answer or self._stopped.is_set():
                self._note(f"are you okay? -> {answer}")
                return answer
        self._note(f"no answer to 'are you okay?' after {ASK_ATTEMPTS} attempts")
        return None

    def _ask_if_hurt(self):
        self._set_state("talking")
        self._speech.say("did_you_get_hurt")
        self._set_state("waiting_answer")
        answer = self._wait_answer("Did you get hurt?", ("yes", "no"), HURT_WAIT_S)
        self._note(f"did you get hurt? -> {answer or 'no answer'}")
        return answer

    def _comfort_after_answers(self) -> None:
        self._set_state("talking")
        if self._hurt == "yes":
            self._speech.say("sorry_hurt")
        elif self._how_are_you is None and self._hurt is None:
            # a silent child may be scared or unwell: get the attention of adults nearby
            self._buzz()
            self._note("child did not answer: buzzer to call adults nearby")
        else:
            self._speech.say("all_good")
        self._speech.say("stay_here")

    def _tell_parents(self) -> None:
        if not self._telegram:
            self._note("no parent channel configured: nobody was told")
            return
        hurt_line = "🚑 Consider calling {}.".format(self._child["emergency_number"])
        text = (
            f"✅ We found {self._name} in the {self._zone} at {_clock(self._found_time)}. "
            f"{self._name} {HOW_ARE_YOU[self._how_are_you]} and {HURT[self._hurt]}. "
            "The robot is with them.\n\nTap the call button to see them now."
            + ("\n" + hurt_line if self._hurt == "yes" else "")
        )
        buttons = (("📹 Video call", self._call_url),) + ETA_BUTTONS
        photo_path = os.path.join(tempfile.gettempdir(), "bobac_care_photo.jpg")
        has_photo = self._take_photo(photo_path)
        for chat_id in self._telegram.chat_ids:
            sent = self._telegram.send(
                chat_id, text, buttons=buttons, photo_path=photo_path if has_photo else None
            )
            self._note(
                "parent message {} ({})".format(
                    "sent" if sent else "FAILED", "with photo" if has_photo else "no photo"
                )
            )

    def _start_call(self) -> None:
        with self._lock:
            self._call_active = True
        if self._show_call:
            self._show_call(self._call_url)
        self._set_state("video_call")
        self._note("video call open on the screen: " + self._call_url)
        self._speech.say_text(f"I am calling your {self._parent}. Look at my screen!")

    def _stay_until_parent_arrives(self) -> None:
        next_calm_at = time.time() + CALM_EVERY_S
        breathe_next = False
        while not self._stopped.is_set():
            if self._telegram:
                events = self._telegram.events(POLL_S)
            else:
                self._stopped.wait(POLL_S)
                events = []
            for event in events:
                if self._handle_parent(event) == "arrived":
                    self._close_parent_arrived()
                    return
            # once a parent replied they are probably talking on the call: stay quiet
            if self._parent_coming is None and time.time() >= next_calm_at:
                if breathe_next:
                    self._breathing_game()
                else:
                    self._speech.say("still_here")
                breathe_next = not breathe_next
                next_calm_at = time.time() + CALM_EVERY_S
            elif self._parent_coming and self._eta_at and time.time() >= next_calm_at:
                self._say_eta()
                next_calm_at = time.time() + CALM_EVERY_S * 3

    def _end_call(self) -> None:
        with self._lock:
            was_active, self._call_active = self._call_active, False
        if was_active and self._show_call:
            self._show_call(None)

    def _close_parent_arrived(self) -> None:
        self._end_call()
        self._speech.say_text(f"Yay, {self._name}! Your family is here.")
        self._note(f"{self._parent_coming} is with {self._name}: closed")
        self._tell_all_parents("✅ Closed.\n\n" + self._report())
        self._set_state("idle")

    # ---------------------------------------------------------------- parents

    def _handle_parent(self, event: dict):
        name = event["name"]
        if event["kind"] == "button" and event["data"].startswith("eta:"):
            minutes = int(event["data"].split(":")[1])
            with self._lock:
                self._parent_coming = name
                self._eta_at = time.time() + minutes * 60
            self._note(f"{name} is on the way, about {minutes} min")
            self._speech.say_text(
                f"{self._name}, {name} is coming and will be here in about {minutes} minutes."
            )
            self._telegram.send(
                event["chat_id"],
                f"I told {self._name}. Text or voice messages you send here are played by "
                "the robot. Press the button when you are with them.",
                buttons=WITH_CHILD_BUTTON,
            )
        elif event["kind"] == "button" and event["data"] == "arrived":
            with self._lock:
                self._parent_coming = name
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

    def _tell_all_parents(self, text: str) -> None:
        if self._telegram:
            for chat_id in self._telegram.chat_ids:
                self._telegram.send(chat_id, text)

    # ---------------------------------------------------------------- comforting

    def _say_eta(self) -> None:
        minutes_left = max(1, math.ceil((self._eta_at - time.time()) / 60))
        self._speech.say_text(
            "{} will be here in about {} {}. I am staying with you.".format(
                self._parent_coming, minutes_left, "minute" if minutes_left == 1 else "minutes"
            )
        )

    def _breathing_game(self) -> None:
        self._speech.say_text(f"{self._name}, let us blow up a big balloon together.")
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
        done = threading.Event()
        if self._listen:
            spoken = {option: SPOKEN_ANSWERS[option] for option in options}
            threading.Thread(
                target=self._answer_by_voice, args=(spoken, timeout_s, done), daemon=True
            ).start()
        self._answered.wait(timeout_s)
        done.set()  # stop listening as soon as there is an answer (or none)
        with self._lock:
            self._question = None
            self._options = []
            return self._answer

    def _answer_by_voice(self, spoken: dict, timeout_s: float, done) -> None:
        heard = self._listen.wait_word(spoken, timeout_s, cancel=done)
        if heard and self.answer(heard):
            self._note("answer heard out loud: " + heard)

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
        minutes = (time.time() - self._found_time) / 60
        found_at = _clock(self._found_time)
        header = (
            f"Report: {self._name} found at {found_at} in the {self._zone}, {minutes:.0f} min ago."
        )
        return "\n".join([header] + lines)


def _clock(epoch_s: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(epoch_s))
