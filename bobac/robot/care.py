"""What the robot does once it reaches the lost child the drone found.

    greet the child by name -> photo -> video call opens on the robot's screen
    -> tell the parents (one "join the video call" button)
    -> keep the child company until stop() is called (the robot cannot see the call end).

The two questions ("are you okay?", "did you get hurt?") are switched off with ASK_QUESTIONS:
reaching the parent comes first, and a voice agent is meant to talk with the child meanwhile.

Plain Python 3.8, no ROS: everything that touches hardware is passed in, so it runs on a laptop.
The HTTP bridge (or care_server.py) owns one CareSession: `run()` in a thread, `answer()` from
the screen buttons, `stop()` to abort, `snapshot()` for /status.
"""

import json
import logging
import os
import tempfile
import threading
import time
import urllib.parse
import uuid

ASK_QUESTIONS = False  # ask "are you okay?" / "did you get hurt?" before calling the parents
ROBOT_KEEPS_COMPANY = True  # calm phrases + counting game; turn off when a voice agent talks
NOTIFY_ATTEMPTS = 3  # the parents must get the message: retry on a flaky connection
NOTIFY_RETRY_S = 5
ASK_ATTEMPTS = 2
ANSWER_WAIT_S = 20
HURT_WAIT_S = 15
POLL_S = 2  # how long one wait for parent events lasts
CALM_EVERY_S = 40
COUNT_TO = 5
COUNT_STEP_S = 1.5  # one number every 1.5 s: slow enough for a child to follow
MAX_SPOKEN_MESSAGE_CHARS = 300
TIMELINE_IN_SNAPSHOT = 8
# Public Jitsi server that lets anyone open a room without logging in (meet.jit.si no longer
# does). The robot and the parent both open the same room URL.
DEFAULT_CALL_BASE_URL = "https://meet.ffmuc.net"
# Only for the robot's own window: hide everything that is not the parent's face, so it is calm
# for a child. The "enter your name" screen is NOT skipped here: skipping it by URL makes the
# server join with microphone and camera off. The call window presses its Join button instead.
ROBOT_CALL_OPTIONS = (
    "config.requireDisplayName=false",
    "config.startWithAudioMuted=false",
    "config.startWithVideoMuted=false",
    "config.toolbarButtons=[]",
    "config.hideConferenceSubject=true",
    "config.hideConferenceTimer=true",
    "config.disableInviteFunctions=true",
    "config.disableReactions=true",
    "config.notifications=[]",
    "config.disableDeepLinking=true",
    "config.enableWelcomePage=false",
)

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
        robot_name = urllib.parse.quote(json.dumps(self._name))
        options = ROBOT_CALL_OPTIONS + ("userInfo.displayName=" + robot_name,)
        self._robot_call_url = self._call_url + "#" + "&".join(options)

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
        self._call_active = False
        self._count = None
        self._timeline = []

    # ---------------------------------------------------------------- called by the bridge

    def run(self) -> None:
        """The whole episode; blocks until stop() is called."""
        self._note(f"robot is with {self._name} in the {self._zone}")
        self._speech.say_text(
            f"Hi {self._name}! I am a friendly robot. Your {self._parent} sent me to find you."
        )
        if ASK_QUESTIONS:
            self._how_are_you = self._ask_how_are_you()
            if self._stopped.is_set():
                return
            self._hurt = self._ask_if_hurt()
            if self._stopped.is_set():
                return
            self._comfort_after_answers()
        # photo before the call (the call takes the camera). The robot enters the call first and
        # the parents are told after, so a parent who joins at once finds the child already there
        photo_path = os.path.join(tempfile.gettempdir(), "bobac_care_photo.jpg")
        has_photo = self._take_photo(photo_path)
        self._start_call()
        self._tell_parents(photo_path if has_photo else None)
        self._keep_company_until_stopped()

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
                    "call_url": self._call_url if self._call_active else None,
                    "count": self._count,
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

    def _tell_parents(self, photo_path) -> None:
        if not self._telegram:
            self._note("no parent channel configured: nobody was told")
            return
        text = (
            f"✅ We found {self._name} in the {self._zone} at {_clock(self._found_time)}. "
            "The robot is with them."
        )
        if ASK_QUESTIONS:
            text += f" {self._name} {HOW_ARE_YOU[self._how_are_you]} and {HURT[self._hurt]}."
            if self._hurt == "yes":
                text += "\n🚑 Consider calling {}.".format(self._child["emergency_number"])
        text += "\n\nJoin the video call to see and talk to them now."
        buttons = (("📹 Join the video call", self._call_url),)
        for chat_id in self._telegram.chat_ids:
            for attempt in range(1, NOTIFY_ATTEMPTS + 1):
                if self._telegram.send(chat_id, text, buttons=buttons, photo_path=photo_path):
                    self._note(f"parent message sent (attempt {attempt})")
                    break
                self._note(f"parent message FAILED (attempt {attempt} of {NOTIFY_ATTEMPTS})")
                if self._stopped.wait(NOTIFY_RETRY_S):
                    return

    def _start_call(self) -> None:
        with self._lock:
            self._call_active = True
        if self._show_call:
            self._show_call(self._robot_call_url)
        self._set_state("video_call")
        self._note("video call open on the screen: " + self._call_url)
        self._speech.say_text(f"I am calling your {self._parent}. Look at my screen!")

    def _keep_company_until_stopped(self) -> None:
        next_calm_at = time.time() + CALM_EVERY_S
        count_next = False
        while not self._stopped.is_set():
            if self._telegram:
                events = self._telegram.events(POLL_S)
            else:
                self._stopped.wait(POLL_S)
                events = []
            for event in events:
                self._play_parent_message(event)
            if ROBOT_KEEPS_COMPANY and time.time() >= next_calm_at:
                if count_next:
                    self._counting_game()
                else:
                    self._speech.say("still_here")
                count_next = not count_next
                next_calm_at = time.time() + CALM_EVERY_S

    def _end_call(self) -> None:
        with self._lock:
            was_active, self._call_active = self._call_active, False
        if was_active and self._show_call:
            self._show_call(None)

    # ---------------------------------------------------------------- parents

    def _play_parent_message(self, event: dict) -> None:
        name = event["name"]
        if event["kind"] == "text":
            self._note("message from {}: {}".format(name, event["text"]))
            spoken = event["text"][:MAX_SPOKEN_MESSAGE_CHARS]
            self._speech.say_text(f"Message from {name}: {spoken}")
        elif event["kind"] == "voice":
            self._note(f"voice message from {name}")
            voice_path = os.path.join(tempfile.gettempdir(), "bobac_care_voice.oga")
            if self._telegram.download(event["file_id"], voice_path):
                self._speech.say_text(f"Voice message from {name}.")
                self._speech.play_file(voice_path)

    # ---------------------------------------------------------------- comforting

    def _counting_game(self) -> None:
        """Count to five together, one finger at a time: simple, and it keeps them busy."""
        self._speech.say("count_intro")
        for number in range(1, COUNT_TO + 1):
            self._set_count(number)
            started = time.time()
            self._speech.say_text(str(number))
            if self._stopped.wait(max(0.0, COUNT_STEP_S - (time.time() - started))):
                break
        self._set_count(None)
        if not self._stopped.is_set():
            self._speech.say("count_done")

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

    def _set_count(self, number) -> None:
        with self._lock:
            self._count = number

    def _note(self, event: str) -> None:
        log.info(event)
        with self._lock:
            self._timeline.append({"time": time.time(), "event": event})


def _clock(epoch_s: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(epoch_s))
