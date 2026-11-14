"""Live Bluetooth LE link to the HoverAir X1: telemetry in, a few commands out.

`DroneLink` runs bleak's asyncio loop in a background thread so the threaded web server and the
CLI can call `snapshot()` and the commands from any thread. Close the Hover app (and turn the
phone's Bluetooth off) first: the drone accepts one Bluetooth central.
Message framing is in adgp.py, the protobuf messages in pbwire.py.
"""

import asyncio
import contextlib
import threading
import time
from collections.abc import Callable
from typing import Any

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from . import adgp, pbwire

SERVICE_UUID = "0000fe45-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # drone -> laptop
WRITE_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # laptop -> drone, write without response
NAME_PREFIX = "HoverX1"
# phone id the owner's app sent in its handshake: one the drone already lists as bound
PHONE_ID = "2121602680692977664"
WIFI_SSID = "HoverX1_060039"
WIFI_COUNTRY = "CH"
DEFAULT_WIFI_PASSWORD = "<DRONE_WIFI_PASSWORD>"  # the team's test password, to be rotated; set on every connect

SCAN_TIMEOUT_S = 15.0
RPC_TIMEOUT_S = 2.0
WIFI_TIMEOUT_S = 10.0
POLL_PERIOD_S = 1.0
MAX_MISSED_POLLS = 3  # polls in a row where the drone answered nothing -> reconnect
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 10.0
MANUAL_MAX_AGE_S = 5.0  # a mode reading older than this no longer unlocks the sticks
THREAD_START_TIMEOUT_S = 5.0
HEARTBEAT_PERIOD_S = 1.0  # the app keeps sending this over Wi-Fi while the preview runs
MAX_MISSED_HEARTBEATS = 5  # the app calls the drone disconnected after this many in a row
MANUAL_CONFIRM_TIMEOUT_S = 3.0  # the app sees the mode push about 0.5 s after the request
MANUAL_CONFIRM_POLL_S = 0.1
MANUAL_STEPS = ("wifi", "preview", "manual", "video")

_POLL_REQUESTS = (
    pbwire.REQ_BATTERY_STATUS,
    pbwire.REQ_GET_FLIGHT_STATUS,
    pbwire.REQ_GET_FLIGHT_MODE,
    pbwire.REQ_CHARGER_STATE,
    pbwire.REQ_GET_STORAGE_CAPACITY,
)


class DroneLink:
    def __init__(
        self,
        name_prefix: str = NAME_PREFIX,
        phone_id: str = PHONE_ID,
        ssid: str = WIFI_SSID,
        country: str = WIFI_COUNTRY,
        wifi_password: str | None = DEFAULT_WIFI_PASSWORD,
    ) -> None:
        self._name_prefix = name_prefix
        self._phone_id = phone_id
        self._ssid = ssid
        self._country = country
        self._wifi_password = wifi_password  # None: leave the drone's Wi-Fi alone on connect
        self._wifi: dict[str, Any] = {"state": "off" if wifi_password is None else "pending"}
        self._heartbeat_stop: threading.Event | None = None
        self._entering_manual = threading.Lock()
        self._manual_session = False  # neutral sticks may flow; see in_manual_session()
        self._manual_steps: list[str] = []
        self._manual_error: str | None = None
        self._thread = threading.Thread(target=self._run_thread, name="drone-ble", daemon=True)
        self._ready = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._main_task: asyncio.Task[None] | None = None

        self._lock = threading.Lock()
        self._values: dict[str, tuple[Any, float]] = {}  # name -> (value, unix time)
        self._connected_since: float | None = None
        self._rssi: int | None = None
        self._last_message: float | None = None
        self._last_error: str | None = None

        # loop-thread only
        self._client: BleakClient | None = None
        self._reassembler = adgp.Reassembler()
        self._waiters: dict[int, asyncio.Future[pbwire.RpcResponse]] = {}
        self._next_id = 1
        self._response_handlers = self._handlers(
            {
                pbwire.RESP_BATTERY_STATUS: ("battery", pbwire.decode_battery),
                pbwire.RESP_CHARGER_STATE: ("charger", pbwire.decode_charger),
                pbwire.RESP_GET_FLIGHT_MODE: ("mode", pbwire.decode_flight_mode),
                pbwire.RESP_GET_FLIGHT_STATUS: ("flight_state", pbwire.decode_flight_state),
                pbwire.RESP_GET_STORAGE_CAPACITY: ("storage", pbwire.decode_storage),
            }
        )
        self._event_handlers = self._handlers(
            {
                pbwire.EVENT_BATTERY_STATUS: ("battery", pbwire.decode_battery),
                pbwire.EVENT_CHARGER: ("charger", pbwire.decode_charger),
                pbwire.EVENT_FLIGHT_MODE_CHANGE: ("mode", pbwire.decode_flight_mode),
                pbwire.EVENT_FLIGHT_STATUS_CHANGE: ("flight_state", pbwire.decode_flight_state),
                pbwire.EVENT_FLIGHT_EXCEPTION: ("exception", pbwire.decode_flight_exception),
                pbwire.EVENT_FLIGHT_ENGINEERING_INFO: ("odometry", pbwire.decode_engineering),
            }
        )

    # --- public, any thread -----------------------------------------------------------------

    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(THREAD_START_TIMEOUT_S):
            raise RuntimeError("Bluetooth thread did not start")

    def stop(self) -> None:
        if self._loop and self._main_task:
            self._loop.call_soon_threadsafe(self._main_task.cancel)
        self._thread.join()

    def snapshot(self) -> dict[str, Any]:
        """Latest state, JSON-ready. Every section has `ts` (unix seconds, None until received)."""
        now = time.time()
        with self._lock:
            values = dict(self._values)
            wifi = dict(self._wifi)
            manual_entry = {
                "session": self._manual_session,
                "steps": list(self._manual_steps),
                "error": self._manual_error,
            }
            since, rssi, last, error = (
                self._connected_since,
                self._rssi,
                self._last_message,
                self._last_error,
            )

        def get(name: str) -> tuple[Any, float | None]:
            return values.get(name, (None, None))

        mode, mode_ts = get("mode")
        state, state_ts = get("flight_state")
        battery, battery_ts = get("battery")
        charger, charger_ts = get("charger")
        storage, storage_ts = get("storage")
        odometry, odometry_ts = get("odometry")
        exception, exception_ts = get("exception")
        return {
            "connection": {
                "connected": since is not None,
                "rssi": rssi,
                "last_message_age_s": None if last is None else round(now - last, 2),
                "error": error,
            },
            "wifi": wifi,
            "manual_entry": manual_entry,
            "mode": {
                "value": mode,
                "name": None if mode is None else pbwire.mode_name(mode),
                "ts": mode_ts,
            },
            "flight_state": {"value": state, "ts": state_ts},
            "battery": {
                "soc": battery.soc if battery else None,
                "voltage": battery.voltage if battery else None,
                "current": battery.current if battery else None,
                "temp": battery.temp if battery else None,
                "ts": battery_ts,
                "charging": charger.connected and not charger.fully_charged if charger else None,
                "charger_connected": charger.connected if charger else None,
                "fully_charged": charger.fully_charged if charger else None,
                "charger_ts": charger_ts,
            },
            "odometry": {
                "position": _xyz(odometry.position) if odometry else None,
                "velocity": _xyz(odometry.velocity) if odometry else None,
                "motors_rpm": odometry.motors_rpm if odometry else None,
                "battery_voltage": odometry.battery_voltage if odometry else None,
                "total_current": odometry.total_current if odometry else None,
                "ts": odometry_ts,
            },
            "storage": {
                "available_bytes": storage.available_bytes if storage else None,
                "total_bytes": storage.total_bytes if storage else None,
                "ts": storage_ts,
            },
            "exception": {
                "errors": exception.errors if exception else None,
                "event": exception.event if exception else None,
                "ts": exception_ts,
            },
        }

    def is_manual(self) -> bool:
        """True only with a live link and a fresh reading of MANUAL mode (stick interlock)."""
        with self._lock:
            mode, ts = self._values.get("mode", (None, 0.0))
            return (
                self._connected_since is not None
                and mode == pbwire.FlightMode.MANUAL
                and time.time() - ts <= MANUAL_MAX_AGE_S
            )

    def in_manual_session(self) -> bool:
        """True after enter_manual() got the drone's heartbeat, until the heartbeat is lost."""
        with self._lock:
            return self._manual_session

    def set_manual(self) -> dict[str, Any]:
        """The bare flag over Bluetooth. The drone drops back to its own mode: use enter_manual."""
        return self._command(pbwire.encode_set_manual_request, RPC_TIMEOUT_S)

    def enter_manual(self) -> dict[str, Any]:
        """What the app does when the user taps Manual Control, in the same order, over Wi-Fi.

        1. heartbeat_info_request, then every second (the app's "drone connected"; its stick
           stream, neutral until moved, starts here: this opens the manual session)
        2. switch_to_manual_capture_request(true), set_udp_server_ipaddress_request(our ip)
        3. set_flight_mode_to_manual_request(true), over Wi-Fi HTTP, not Bluetooth
        4. camera_start_preview_request(enable, session 0): the drone starts pushing video
        Needs the laptop on the drone's Wi-Fi. Progress is in snapshot()["manual_entry"].
        """
        address = pbwire.drone_network_ip()
        if address is None:
            password = self._wifi_password or DEFAULT_WIFI_PASSWORD
            return self._manual_failed(f"join {self._ssid} (password {password}) first")
        if not self._entering_manual.acquire(blocking=False):
            return {"ok": False, "error": "manual entry is already running"}
        try:
            return self._enter_manual(address)
        finally:
            self._entering_manual.release()

    def land(self) -> dict[str, Any]:
        """abort_flight_request: the drone lands."""
        return self._command(
            lambda rid: pbwire.encode_rpc_request(rid, pbwire.REQ_ABORT_FLIGHT), RPC_TIMEOUT_S
        )

    def start_wifi(self, password: str) -> dict[str, Any]:
        """Start the drone's Wi-Fi access point, which carries video and sticks."""
        result = self._command(self._wifi_request(password), WIFI_TIMEOUT_S)
        self._set_wifi(result)
        if result["ok"]:
            self._wifi_password = password  # what the "join the Wi-Fi" hint should say
        return result

    def start_video(self) -> dict[str, Any]:
        """Ask the drone to push live video to this laptop's address on its Wi-Fi (UDP 7078).

        enter_preview_mode_request over the Wi-Fi RPC: that is where the app sends it. Needs the
        laptop joined to the drone's Wi-Fi.
        """
        address = pbwire.drone_network_ip()
        if address is None:
            return {"ok": False, "error": "laptop is not on the drone's Wi-Fi"}
        result = _wifi_rpc(
            lambda rid: pbwire.encode_enter_preview_request(rid, address, pbwire.VIDEO_CODEC_AVC)
        )
        if result["ok"]:
            self._start_heartbeat()
        return result

    def stop_video(self) -> dict[str, Any]:
        self._stop_heartbeat()
        self._set_manual_session(False)
        stopped = _wifi_rpc(
            lambda rid: pbwire.encode_stop_preview_request(rid, pbwire.VIDEO_CODEC_AVC)
        )
        if not stopped["ok"]:
            return stopped
        return _wifi_rpc(lambda rid: pbwire.encode_rpc_request(rid, pbwire.REQ_EXIT_PREVIEW_MODE))

    def _enter_manual(self, address: str) -> dict[str, Any]:
        with self._lock:
            self._manual_steps = []
            self._manual_error = None
        codec = pbwire.VIDEO_CODEC_AVC
        plan = (
            (
                MANUAL_STEPS[0],
                (lambda rid: pbwire.encode_rpc_request(rid, pbwire.REQ_HEARTBEAT_INFO),),
            ),
            (
                MANUAL_STEPS[1],
                (
                    pbwire.encode_switch_to_manual_capture_request,
                    lambda rid: pbwire.encode_set_udp_server_request(rid, address),
                ),
            ),
            (MANUAL_STEPS[2], (pbwire.encode_set_manual_request,)),
            (
                MANUAL_STEPS[3],
                (lambda rid: pbwire.encode_camera_start_preview_request(rid, codec),),
            ),
        )
        for step, requests in plan:
            for build in requests:
                result = _wifi_rpc(build)
                if not result["ok"]:
                    return self._manual_failed(f"{step}: {result['error']}")
            with self._lock:
                self._manual_steps.append(step)
            if step == MANUAL_STEPS[0]:
                self._start_heartbeat()
                self._set_manual_session(True)
        if not self._wait_for_manual():
            mode = self.snapshot()["mode"]["name"] or "no mode"
            return self._manual_failed(
                f"requests accepted but the drone reports {mode}, not MANUAL"
            )
        return {"ok": True, "error": None}

    def _wait_for_manual(self) -> bool:
        deadline = time.monotonic() + MANUAL_CONFIRM_TIMEOUT_S
        while not self.is_manual():
            if time.monotonic() >= deadline:
                return False
            time.sleep(MANUAL_CONFIRM_POLL_S)
        return True

    def _manual_failed(self, error: str) -> dict[str, Any]:
        """Close the session (fail closed: no more neutral stream) and keep the reason."""
        with self._lock:
            self._manual_error = error
            self._manual_session = False
        return {"ok": False, "error": error}

    def _set_manual_session(self, active: bool) -> None:
        with self._lock:
            self._manual_session = active

    def _wifi_request(self, password: str) -> Callable[[int], bytes]:
        return lambda rid: pbwire.encode_wifi_start_request(
            rid, self._ssid, password, self._country
        )

    def _set_wifi(self, result: dict[str, Any]) -> None:
        with self._lock:
            if result["ok"]:
                self._wifi = {"state": "on", "ssid": self._ssid}
            else:
                self._wifi = {"state": "error", "error": result["error"]}

    def _start_heartbeat(self) -> None:
        if self._heartbeat_stop is not None:
            return
        stop = threading.Event()
        self._heartbeat_stop = stop
        threading.Thread(
            target=_heartbeat,
            args=(stop, lambda: self._set_manual_session(False)),
            name="drone-heartbeat",
            daemon=True,
        ).start()

    def _stop_heartbeat(self) -> None:
        if self._heartbeat_stop is not None:
            self._heartbeat_stop.set()
            self._heartbeat_stop = None

    # --- command plumbing -------------------------------------------------------------------

    def _command(self, build: Callable[[int], bytes], timeout: float) -> dict[str, Any]:
        if self._loop is None or self._connected_since is None:
            return {"ok": False, "error": "not connected"}
        future = asyncio.run_coroutine_threadsafe(self._call(build, timeout), self._loop)
        try:
            response = future.result(timeout + RPC_TIMEOUT_S)
        except (TimeoutError, BleakError, OSError) as exc:
            future.cancel()
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        return {"ok": response.error is None, "response": response.kind, "error": response.error}

    # --- loop thread ------------------------------------------------------------------------

    def _run_thread(self) -> None:
        with contextlib.suppress(asyncio.CancelledError):
            asyncio.run(self._main())

    async def _main(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._main_task = asyncio.current_task()
        self._ready.set()
        backoff = RECONNECT_MIN_S
        while True:
            try:
                await self._session()
            except (BleakError, OSError, TimeoutError, ConnectionError) as exc:
                self._set_error(f"{type(exc).__name__}: {exc}")
            was_up = self._connected_since is not None
            backoff = RECONNECT_MIN_S if was_up else min(backoff * 2, RECONNECT_MAX_S)
            with self._lock:
                self._connected_since = None
            await asyncio.sleep(backoff)

    async def _session(self) -> None:
        device = await self._find_device()
        dropped = asyncio.Event()
        loop = asyncio.get_running_loop()
        async with BleakClient(
            device, disconnected_callback=lambda _c: loop.call_soon_threadsafe(dropped.set)
        ) as client:
            self._client = client
            self._reassembler = adgp.Reassembler()
            await client.start_notify(NOTIFY_UUID, self._on_notify)
            handshake = await self._call(
                lambda rid: pbwire.encode_handshake_request(rid, self._phone_id), RPC_TIMEOUT_S
            )
            if handshake.error:
                raise ConnectionError(f"handshake refused: {handshake.error}")
            with self._lock:
                self._connected_since = time.time()
                self._last_error = None
            await self._autostart_wifi()
            await self._poll_until(dropped)
        self._client = None

    async def _autostart_wifi(self) -> None:
        """The drone's Wi-Fi is off after a restart, so switch it on after every handshake."""
        if self._wifi_password is None:
            return
        try:
            response = await self._call(self._wifi_request(self._wifi_password), WIFI_TIMEOUT_S)
            result = {"ok": response.error is None, "error": response.error}
        except TimeoutError:
            result = {"ok": False, "error": "no answer to wifi_start_request"}
        self._set_wifi(result)

    async def _find_device(self) -> Any:
        rssi: list[int] = []

        def match(device: Any, advertisement: Any) -> bool:
            found = SERVICE_UUID in [u.lower() for u in advertisement.service_uuids] or (
                advertisement.local_name or device.name or ""
            ).startswith(self._name_prefix)
            if found:
                rssi[:] = [advertisement.rssi]
            return found

        device = await BleakScanner.find_device_by_filter(match, timeout=SCAN_TIMEOUT_S)
        if device is None:
            raise ConnectionError("drone not found (powered on, Hover app closed?)")
        with self._lock:
            self._rssi = rssi[0] if rssi else None
        return device

    async def _poll_until(self, dropped: asyncio.Event) -> None:
        missed = 0
        while not dropped.is_set():
            answered = 0
            for request_field in _POLL_REQUESTS:
                try:
                    await self._call(
                        lambda rid, f=request_field: pbwire.encode_rpc_request(rid, f),
                        RPC_TIMEOUT_S,
                    )
                    answered += 1
                except TimeoutError:
                    continue
            missed = 0 if answered else missed + 1
            if missed >= MAX_MISSED_POLLS:
                raise ConnectionError("drone stopped answering")
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(dropped.wait(), POLL_PERIOD_S)

    async def _call(self, build: Callable[[int], bytes], timeout: float) -> pbwire.RpcResponse:
        if self._client is None:
            raise ConnectionError("not connected")
        request_id = self._next_id
        self._next_id += 1
        future: asyncio.Future[pbwire.RpcResponse] = asyncio.get_running_loop().create_future()
        self._waiters[request_id] = future
        try:
            # the app numbers frames with the request id minus one
            for frame in adgp.encode(build(request_id), (request_id - 1) & 0xFF):
                await self._client.write_gatt_char(WRITE_UUID, frame, response=False)
            return await asyncio.wait_for(future, timeout)
        finally:
            del self._waiters[request_id]

    def _on_notify(self, _characteristic: Any, data: bytearray) -> None:
        try:
            message = self._reassembler.feed(bytes(data))
            if message is None:
                return
            with self._lock:
                self._last_message = time.time()
            if message.type == adgp.TYPE_RPC:
                self._on_response(pbwire.decode_rpc_response(message.payload))
            elif message.type == adgp.TYPE_PUSH:
                self._on_push(message.payload)
        except ValueError as exc:
            self._set_error(f"bad message: {exc}")

    def _on_response(self, response: pbwire.RpcResponse) -> None:
        handler = self._response_handlers.get(response.kind)
        if handler and response.error is None:
            handler(response.body)
        waiter = self._waiters.get(response.request_id)
        if waiter and not waiter.done():
            waiter.set_result(response)

    def _on_push(self, payload: bytes) -> None:
        event = pbwire.decode_push(payload)
        handler = self._event_handlers.get(event[0]) if event else None
        if event and handler:
            handler(event[1])

    def _handlers(
        self, table: dict[int, tuple[str, Callable[[bytes], Any]]]
    ) -> dict[int, Callable[[bytes], None]]:
        def make(name: str, decode: Callable[[bytes], Any]) -> Callable[[bytes], None]:
            return lambda body: self._store(name, decode(body))

        return {number: make(name, decode) for number, (name, decode) in table.items()}

    def _store(self, name: str, value: Any) -> None:
        with self._lock:
            self._values[name] = (value, time.time())

    def _set_error(self, text: str) -> None:
        with self._lock:
            self._last_error = text


def _wifi_rpc(build: Callable[[int], bytes]) -> dict[str, Any]:
    try:
        response = pbwire.post_wifi_rpc(build(pbwire.WIFI_REQUEST_ID))
    except OSError as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    return {"ok": response.error is None, "response": response.kind, "error": response.error}


def _heartbeat(stop: threading.Event, on_lost: Callable[[], None]) -> None:
    """The app's 1 Hz heartbeat; `on_lost` runs on every beat after MAX_MISSED_HEARTBEATS misses."""
    missed = 0
    while not stop.wait(HEARTBEAT_PERIOD_S):
        beat = _wifi_rpc(lambda rid: pbwire.encode_rpc_request(rid, pbwire.REQ_HEARTBEAT_INFO))
        missed = 0 if beat["ok"] else missed + 1
        if missed >= MAX_MISSED_HEARTBEATS:
            on_lost()


def _xyz(values: tuple[float, float, float]) -> dict[str, float]:
    return dict(zip("xyz", values, strict=True))
