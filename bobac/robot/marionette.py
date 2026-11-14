"""Run a script inside a Firefox started with `--marionette` (the robot's call window).

Firefox's Marionette protocol, on localhost, with the standard library only (robot, Python 3.8):
each message is `<length>:<json>`; a command is [0, id, name, params] and its answer is
[1, id, error, result].
"""

import contextlib
import json
import socket

SOCKET_TIMEOUT_S = 5


class Marionette:
    def __init__(self, port: int):
        self._port = port
        self._socket = None
        self._next_id = 0

    def run_script(self, script: str):
        """Run `script` in the current page and return what it returns. Raises OSError while
        Firefox is not reachable (still starting, or closed)."""
        try:
            if self._socket is None:
                self._connect()
            return self._command("WebDriver:ExecuteScript", {"script": script, "args": []})["value"]
        except (ValueError, KeyError, TypeError) as error:
            self.close()
            raise OSError(f"marionette: {error}") from error
        except OSError:
            self.close()
            raise

    def close(self) -> None:
        if self._socket is not None:
            with contextlib.suppress(OSError):
                self._socket.close()
            self._socket = None

    def _connect(self) -> None:
        self._socket = socket.create_connection(("127.0.0.1", self._port), SOCKET_TIMEOUT_S)
        self._socket.settimeout(SOCKET_TIMEOUT_S)
        self._read()  # Firefox greets first
        self._command("WebDriver:NewSession", {})

    def _command(self, name: str, params: dict) -> dict:
        self._next_id += 1
        body = json.dumps([0, self._next_id, name, params])
        self._socket.sendall(f"{len(body.encode())}:{body}".encode())
        _, _, error, result = self._read()
        if error:
            raise ValueError(error.get("message", "marionette error"))
        return result

    def _read(self):
        length = b""
        while not length.endswith(b":"):
            byte = self._socket.recv(1)
            if not byte:
                raise OSError("firefox closed the connection")
            length += byte
        remaining = int(length[:-1])
        body = b""
        while len(body) < remaining:
            chunk = self._socket.recv(remaining - len(body))
            if not chunk:
                raise OSError("firefox closed the connection")
            body += chunk
        return json.loads(body)
