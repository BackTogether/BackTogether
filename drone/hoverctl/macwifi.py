"""Join the drone's Wi-Fi from the laptop, so Manual control needs one click.

macOS uses `networksetup`, Linux uses NetworkManager's `nmcli`. Elsewhere (e.g. Windows) the
user joins by hand. The drone's access point only exists after the Bluetooth link sent
wifi_start (ble.py), and it has no internet, so the OS never joins it on its own.
"""

import re
import shutil
import subprocess
import sys
import time

from .pbwire import drone_network_ip

JOIN_TIMEOUT_S = 25.0  # association plus DHCP on the drone's access point
POLL_S = 0.5
_WIFI_PORT = re.compile(r"Hardware Port: (?:Wi-Fi|AirPort)\nDevice: (\S+)")


def join(ssid: str, password: str) -> str | None:
    """Join `ssid` and wait for an address on the drone's network. Returns an error or None."""
    if sys.platform == "darwin":
        message = _join_macos(ssid, password)
    elif sys.platform.startswith("linux"):
        message = _join_linux(ssid, password)
    else:
        return f"join {ssid} (password {password}) by hand: auto-join is macOS and Linux only"
    deadline = time.monotonic() + JOIN_TIMEOUT_S
    while time.monotonic() < deadline:
        if drone_network_ip():
            return None
        time.sleep(POLL_S)
    return f"could not join {ssid}: {message or 'no address from the drone'}"


def _join_macos(ssid: str, password: str) -> str:
    device = _macos_wifi_device()
    if device is None:
        return "no Wi-Fi interface found"
    result = subprocess.run(
        ["networksetup", "-setairportnetwork", device, ssid, password],
        capture_output=True,
        text=True,
        check=False,
    )
    # networksetup exits 0 even when it fails; it only prints the reason
    return (result.stdout + result.stderr).strip()


def _macos_wifi_device() -> str | None:
    ports = subprocess.run(
        ["networksetup", "-listallhardwareports"], capture_output=True, text=True, check=False
    ).stdout
    match = _WIFI_PORT.search(ports)
    return match.group(1) if match else None


def _join_linux(ssid: str, password: str) -> str:
    if not shutil.which("nmcli"):
        return "nmcli (NetworkManager) not found: join the Wi-Fi by hand"
    # A fresh scan, so an access point that just started (wifi_start) is in the list
    subprocess.run(["nmcli", "device", "wifi", "rescan"], capture_output=True, check=False)
    result = subprocess.run(
        ["nmcli", "device", "wifi", "connect", ssid, "password", password],
        capture_output=True,
        text=True,
        check=False,
    )
    return "" if result.returncode == 0 else (result.stdout + result.stderr).strip()
