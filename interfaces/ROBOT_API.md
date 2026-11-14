# interfaces/ROBOT_API.md: robot contract (issue #18)

Lets P1-P4 work in parallel. Code against this; until the real piece exists, use the fake
with the same signature in [robot_fakes.py](robot_fakes.py). **Do not change this on your own:
ask on #18.** Only additions with defaults.

All Python here runs **on the robot** (system Python 3.8, plain dicts, JSON-compatible).
Units: metres, radians, seconds. Poses in frame `map`; bearings in `base_footprint`, ROS
convention (0 = straight ahead, positive = left).

## Names
- **Places / zones** (P1 owns `bobac/config/places.yaml`): `sofa`, `armchair`, `kitchen`, `base`.
  A fall zone name is a place name.
- **Phrases** (P3 owns the audio): `greeting`, `are_you_ok`, `press_button`, `all_good`,
  `calling_help`, `help_on_way`.

## Python APIs (one module per owner, in `bobac/robot/`)
| Owner | Module | Function | Returns |
|---|---|---|---|
| P1 | `navigator.py` | `go_to(place: str, timeout_s: float = 120.0)` blocking | `{"success": bool, "message": str}`; unknown place -> `success: False, message: "unknown place"` |
| P1 | `navigator.py` | `stop()` cancels any goal | `None` |
| P1 | `navigator.py` | `current_pose()` | `{"x": float, "y": float, "yaw": float}` |
| P4 | `person_search.py` | `find_person(timeout_s: float = 30.0)` | `{"found": bool, "bearing_rad": float, "distance_m": float or None}` |
| P4 | `person_search.py` | `approach(distance_m: float = 0.6)` | `{"success": bool, "message": str}` |
| P4 | `fall_check.py` | `is_fallen()` | `bool` |
| P3 | `speech.py` | `say(phrase_id: str)` blocking until played | `bool` (`False` if unknown id) |

Rules for these modules:
- **The bridge calls `rospy.init_node("bridge")` once.** Modules never call it; create
  publishers, subscribers and action/service clients lazily (on first use), not at import.
- Functions may be called from any thread (the bridge serves HTTP with one thread per request).
- No speed above 0.4 m/s / 0.8 rad/s. Only one publisher on `/cmd_vel` at a time.

Using the fakes until the real module exists:
```python
import sys
sys.path.insert(0, "<repo>/interfaces")
import robot_fakes as navigator  # same functions as the real module
```

## P2 · HTTP bridge (`bobac/robot/bridge.py`, port 8000, JSON in/out)
| Method | Body | Reply |
|---|---|---|
| `POST /goto` | `{"place": str}` | `202 {"accepted": true}`; progress in `/status` |
| `POST /stop` | - | `200 {"ok": true}`: stops navigation and any running flow, state back to `idle` |
| `POST /say` | `{"phrase_id": str}` | `200 {"ok": bool}` |
| `POST /fall` | `{"zone": str}` | `202 {"accepted": true}`: starts the flow below (the drone uses it too) |
| `POST /ui/answer` | `{"answer": "ok" or "help"}` | `200 {"ok": true}` |
| `GET /status` | - | see below |
| `GET /ui/...` | - | static files of the screen page (P3, `bobac/robot/ui/`) |

- Bad input -> `400 {"error": str}`. `/goto` or `/fall` while another one runs ->
  `409 {"error": "busy"}` (send `/stop` first).
- Every reply carries `Access-Control-Allow-Origin: *`, so a page served from a laptop can call
  the bridge.

`GET /status`:
```json
{"state": "idle", "place": null, "person_found": false, "is_fallen": null, "answer": null,
 "battery_v": 24.5, "estop_pressed": false, "bumper_hit": false, "smoke": false,
 "temperature_c": 24.5, "message": "", "updated_at": 1763160000.0}
```
- `state` is one of `idle`, `navigating`, `arrived`, `searching`, `approaching`, `talking`,
  `waiting_answer`, `escalated`, `error`.
- `estop_pressed` is `car_data.connect_status == False`; `bumper_hit` is any `car_data.crash`.
  Health fields are `null` until the first `/car_data` message arrives.
- `updated_at` is the time of the last state change (seconds since the epoch).

## Flow (`POST /fall {"zone": z}`)
1. `navigating`: `go_to(z)`. Fails -> `error`.
2. `searching`: `find_person()`. Not found -> escalate (step 6).
3. `approaching`: `approach()`. Fails -> escalate.
4. `talking`: store `is_fallen()`, then `say("are_you_ok")` and `say("press_button")`.
5. `waiting_answer`: up to 30 s for `/ui/answer`.
   `ok` -> `say("all_good")`, back to `idle`.
6. `help`, timeout or a failed step 2-3 -> `say("calling_help")`, buzzer, `escalated`.
   The laptop side sends the caregiver alert when it sees `escalated`.
   It stays `escalated` until `/stop`.

## Fake robot for laptops
`uv run python bobac/mock_robot.py` serves the same HTTP API and `/status` shape on port 8000,
using [robot_fakes.py](robot_fakes.py) (each step takes a few seconds).
