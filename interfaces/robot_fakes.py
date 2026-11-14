"""Fake versions of the P1/P3/P4 robot APIs in ROBOT_API.md, same signatures and return shapes.

Use them until the real module exists, on a laptop or on the robot (plain Python 3.8, no ROS):

    import robot_fakes as navigator
    navigator.go_to("sofa")

Every step only waits a little and logs what the real robot would do.
"""

import logging
import threading

PLACES = ("sofa", "armchair", "kitchen", "base")
PHRASES = ("greeting", "are_you_ok", "press_button", "all_good", "calling_help", "help_on_way")

GO_TO_S = 3.0
FIND_PERSON_S = 2.0
APPROACH_S = 2.0
SAY_S = 1.0
FAKE_PERSON = {"found": True, "bearing_rad": 0.3, "distance_m": 1.5}

log = logging.getLogger("robot_fakes")
_stop_requested = threading.Event()
_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}


def _result(success: bool, message: str) -> dict:
    return {"success": success, "message": message}


def _wait(seconds: float) -> bool:
    """Wait like a real action would; True if stop() interrupted it."""
    _stop_requested.clear()
    return _stop_requested.wait(seconds)


# P1 · navigator.py
def go_to(place: str, timeout_s: float = 120.0) -> dict:
    if place not in PLACES:
        return _result(False, "unknown place")
    log.info("fake go_to(%s)", place)
    if _wait(min(GO_TO_S, timeout_s)):
        return _result(False, "stopped")
    if timeout_s < GO_TO_S:
        return _result(False, "timeout")
    _pose["x"] = float(PLACES.index(place))
    return _result(True, "arrived at " + place)


def stop() -> None:
    log.info("fake stop()")
    _stop_requested.set()


def current_pose() -> dict:
    return dict(_pose)


# P4 · person_search.py
def find_person(timeout_s: float = 30.0) -> dict:
    log.info("fake find_person()")
    if _wait(min(FIND_PERSON_S, timeout_s)):
        return {"found": False, "bearing_rad": 0.0, "distance_m": None}
    return dict(FAKE_PERSON)


def approach(distance_m: float = 0.6) -> dict:
    log.info("fake approach(%.2f)", distance_m)
    if _wait(APPROACH_S):
        return _result(False, "stopped")
    return _result(True, f"at {distance_m:.2f} m")


# P4 · fall_check.py
def is_fallen() -> bool:
    return True


# P3 · speech.py
def say(phrase_id: str) -> bool:
    if phrase_id not in PHRASES:
        return False
    log.info("fake say(%s)", phrase_id)
    _wait(SAY_S)
    return True
