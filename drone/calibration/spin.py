"""Spin-calibrate the yaw rate: one exact revolution per stick value, then land.

drone/patrol/live spin [100 80 60]
(= uv run --offline --no-project --with numpy --with pillow python -m drone.calibration.spin)

Needs drone/fly running, Manual control done, the drone FLYING (hovering in open space with
>1.5 m clear all around). For each stick value it turns right at that constant stick, keeps
the first frame as reference and, after ~70 % of a turn, compares every new frame with it:
the sideways shift between them crosses zero exactly when the view repeats = 360 deg. It
stops there, logs the time and goes on; after the last one it LANDS.
Ctrl+C sends STOP and lands. Log: recordings/spin_<time>.json
"""

import io
import json
import os
import sys
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image

URL = os.environ.get("HOVER_URL", "http://127.0.0.1:8077")
EXPECTED_DPS_PER_PCT = 0.24  # spin-calibrated 2026-11-14; only sizes the search window
SEARCH_FROM = 0.3  # of the expected revolution: start comparing (early, never miss turn 1)
GIVE_UP_AFTER = float(
    os.environ.get("SPIN_GIVE_UP", "2.2")
)  # of the expected revolution: no match by then -> stop this one
SEND_S = 0.05  # stick posts (the app goes neutral 0.3 s after the last one)
SIZE = (160, 90)
MAX_SHIFT = 40  # px at SIZE, ~ +-18 deg
MIN_MATCH = 0.6  # median tile correlation (edges): real repeats scored 0.84-0.88, false 0.4
TILES_X, TILES_Y = 4, 2  # vote over tiles: people in a few tiles are outvoted
PAUSE_S = 3.0
TAB = "spin"


def call(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(URL + path, data, {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=3) as reply:
            raw = reply.read()
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as error:
        return {"ok": False, "status": error.code, "error": error.read().decode()}


def edges(gray: np.ndarray) -> np.ndarray:
    """Gradient magnitude: matches structure (walls, windows, lines), not brightness."""
    gx = np.zeros_like(gray)
    gy = np.zeros_like(gray)
    gx[:, 1:] = np.diff(gray, axis=1)
    gy[1:, :] = np.diff(gray, axis=0)
    magnitude = np.hypot(gx, gy)
    return magnitude - magnitude.mean()


def frame() -> np.ndarray:
    with urllib.request.urlopen(URL + "/frame.jpg", timeout=3) as reply:
        image = Image.open(io.BytesIO(reply.read())).convert("L").resize(SIZE)
    return edges(np.asarray(image, dtype=np.float32))


def _ncc(a: np.ndarray, b: np.ndarray) -> float:
    a, b = a - a.mean(), b - b.mean()
    return float((a * b).sum() / (np.sqrt((a * a).sum() * (b * b).sum()) + 1e-6))


def best_shift(reference: np.ndarray, current: np.ndarray) -> tuple[int, float]:
    """Horizontal shift (px) aligning current onto reference, and its match score.

    The overlap is cut into TILES_X x TILES_Y tiles and a shift scores the MEDIAN tile
    correlation, so people walking through a few tiles don't move the match.
    """
    best = (0, -1.0)
    for shift in range(-MAX_SHIFT, MAX_SHIFT + 1):
        a = reference[:, max(0, shift) : SIZE[0] + min(0, shift)]
        b = current[:, max(0, -shift) : SIZE[0] + min(0, -shift)]
        rows = np.array_split(np.arange(a.shape[0]), TILES_Y)
        cols = np.array_split(np.arange(a.shape[1]), TILES_X)
        scores = [_ncc(a[np.ix_(r, c)], b[np.ix_(r, c)]) for r in rows for c in cols]
        score = float(np.median(scores))
        if score > best[1]:
            best = (shift, score)
    return best


def turn(yaw: float) -> None:
    call(f"/api/sticks?tab={TAB}", {"yaw": yaw})


def spin(stick: float) -> dict:
    call("/speed", {"max": stick})
    expected = 360 / (EXPECTED_DPS_PER_PCT * stick)
    reference = frame()
    print(f"stick {stick:.0f}: spinning right, one turn expected in ~{expected:.0f} s", flush=True)
    start = time.time()
    last_check, previous = 0.0, None
    while True:
        turn(1.0)
        elapsed = time.time() - start
        if elapsed > GIVE_UP_AFTER * expected:
            turn(0.0)
            print("  no match found: view never repeated (low texture or drift?)", flush=True)
            return {"stick": stick, "start": start, "end": time.time(), "revolution_s": None}
        if elapsed > SEARCH_FROM * expected and elapsed - last_check > 0.15:
            last_check = elapsed
            shift, score = best_shift(reference, frame())
            if score >= MIN_MATCH:
                if (
                    previous and previous[1] != 0 and previous[1] * shift <= 0
                ):  # crossed the start view
                    t0, s0 = previous[0], previous[1]
                    exact = t0 + (elapsed - t0) * s0 / (s0 - shift) if s0 != shift else elapsed
                    turn(0.0)
                    rate = 360 / exact
                    print(
                        f"  full turn in {exact:.1f} s -> {rate:.2f} deg/s at stick {stick:.0f}"
                        f" (match {score:.2f})",
                        flush=True,
                    )
                    return {
                        "stick": stick,
                        "start": start,
                        "revolution_s": exact,
                        "dps": rate,
                        "match": score,
                    }
                previous = (elapsed, shift)
        time.sleep(SEND_S)


def save(log: dict, path: str) -> None:
    with open(path, "w") as file:
        json.dump(log, file, indent=1)


def main() -> None:
    sticks = [float(arg) for arg in sys.argv[1:]] or [100.0, 80.0, 60.0]
    status = call("/status")
    telemetry = status.get("telemetry", {})
    if not telemetry.get("in_flight") or telemetry.get("sticks_gate") != "live":
        sys.exit(
            f"not ready: in_flight={telemetry.get('in_flight')} "
            f"gate={telemetry.get('sticks_gate')} (Manual control, then the drone's button)"
        )
    old_max = status.get("speed", {}).get("max", 60)
    out = f"recordings/spin_{time.strftime('%Y%m%d-%H%M%S')}.json"
    log = {"segments": []}
    try:
        for stick in sticks:
            log["segments"].append(spin(stick))
            save(log, out)
            time.sleep(PAUSE_S)
    except KeyboardInterrupt:
        call("/stop", {})
        print("\nSTOP sent", flush=True)
    finally:
        turn(0.0)
        call("/speed", {"max": old_max})
        save(log, out)
        print(f"log: {out}. Landing...", flush=True)
        print("land:", call("/land", {}), flush=True)
        rates = [(s["stick"], s.get("dps")) for s in log["segments"] if s.get("dps")]
        if rates:
            print("result (stick -> deg/s):", ", ".join(f"{s:.0f}->{r:.2f}" for s, r in rates))


if __name__ == "__main__":
    main()
