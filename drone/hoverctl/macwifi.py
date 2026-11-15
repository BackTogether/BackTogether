"""Join the drone's Wi-Fi from the laptop, so Manual control needs one click.

macOS uses `networksetup`, Linux uses NetworkManager's `nmcli`, Windows uses `netsh wlan`.
Elsewhere the user joins by hand. The drone's access point only exists after the Bluetooth link sent
wifi_start (ble.py), and it has no internet, so the OS never joins it on its own.
"""

import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from xml.sax.saxutils import escape

from .pbwire import drone_network_ip

JOIN_TIMEOUT_S = 25.0  # association plus DHCP on the drone's access point
POLL_S = 0.5
_WIFI_PORT = re.compile(r"Hardware Port: (?:Wi-Fi|AirPort)\nDevice: (\S+)")
# Windows only connects to a network it has a profile for. Auto mode and the first place in the
# profile order: if the link drops in flight, Windows rejoins the drone, not a known network with
# internet (it did, 2026-11-15, and the manual session and the sticks were lost).
_NETSH_FIRST_VALUE = re.compile(r"^[ \t]+[^:\n]+?[ \t]*:[ \t]*(\S.*)$", re.MULTILINE)
_WINDOWS_PROFILE = """<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{ssid}</name>
  <SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>auto</connectionMode>
  <MSM><security>
    <authEncryption>
      <authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX>
    </authEncryption>
    <sharedKey>
      <keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{password}</keyMaterial>
    </sharedKey>
  </security></MSM>
</WLANProfile>
"""


def join(ssid: str, password: str) -> str | None:
    """Join `ssid` and wait for an address on the drone's network. Returns an error or None."""
    if sys.platform == "darwin":
        message = _join_macos(ssid, password)
    elif sys.platform.startswith("linux"):
        message = _join_linux(ssid, password)
    elif sys.platform == "win32":
        message = _join_windows(ssid, password)
    else:
        return f"join {ssid} (password {password}) by hand: no auto-join on {sys.platform}"
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


def _join_windows(ssid: str, password: str) -> str:
    with tempfile.TemporaryDirectory() as folder:
        profile = Path(folder) / "drone-wifi.xml"
        profile.write_text(
            _WINDOWS_PROFILE.format(ssid=escape(ssid), password=escape(password)), encoding="utf-8"
        )
        added = _netsh("add", "profile", f"filename={profile}", "user=current")
    # a failed add (e.g. an older profile of the same name) does not stop the connect
    interface = _windows_wifi_interface()
    if interface:
        _netsh("set", "profileorder", f"name={ssid}", f"interface={interface}", "priority=1")
    result = _netsh("connect", f"name={ssid}", f"ssid={ssid}")
    if result.returncode == 0:
        return ""
    return " ".join(part.strip() for part in (added.stdout, result.stdout, result.stderr) if part)


def _windows_wifi_interface() -> str | None:
    # The first "key : value" line is the interface's name, whatever the system's language
    match = _NETSH_FIRST_VALUE.search(_netsh("show", "interfaces").stdout)
    return match.group(1).strip() if match else None


def _netsh(*arguments: str) -> subprocess.CompletedProcess:
    # errors="replace": netsh answers in the console's code page, in the system's language
    return subprocess.run(
        ["netsh", "wlan", *arguments], capture_output=True, text=True, errors="replace", check=False
    )