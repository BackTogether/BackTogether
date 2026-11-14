"""Join the drone's Wi-Fi from macOS, so Manual control needs one click.

The drone's access point only exists after the Bluetooth link sent wifi_start (ble.py), and
it has no internet, so macOS never joins it on its own.
"""

import re
import subprocess
import sys
import time

from .pbwire import drone_network_ip

JOIN_TIMEOUT_S = 25.0  # association plus DHCP on the drone's access point
POLL_S = 0.5
_WIFI_PORT = re.compile(r"Hardware Port: (?:Wi-Fi|AirPort)\nDevice: (\S+)")


def join(ssid: str, password: str) -> str | None:
    """Join `ssid` and wait for an address on the drone's network. Returns an error or None."""
    if sys.platform != "darwin":
        return f"join {ssid} (password {password}) by hand: auto-join is macOS only"
    device = _wifi_device()
    if device is None:
        return "no Wi-Fi interface found"
    result = subprocess.run(
        ["networksetup", "-setairportnetwork", device, ssid, password],
        capture_output=True,
        text=True,
        check=False,
    )
    # networksetup exits 0 even when it fails; it only prints the reason
    message = (result.stdout + result.stderr).strip()
    deadline = time.monotonic() + JOIN_TIMEOUT_S
    while time.monotonic() < deadline:
        if drone_network_ip():
            return None
        time.sleep(POLL_S)
    return f"could not join {ssid}: {message or 'no address from the drone'}"


def _wifi_device() -> str | None:
    ports = subprocess.run(
        ["networksetup", "-listallhardwareports"], capture_output=True, text=True, check=False
    ).stdout
    match = _WIFI_PORT.search(ports)
    return match.group(1) if match else None
