"""Read the keys from the package's hidden .env file into the environment.

The launch file points at it (`env_file` argument, default `$(find kid_care)/.env`), because
roslaunch cannot source a shell file. One `NAME=value` per line, `#` comments and blank lines
ignored, surrounding quotes stripped. A name that is already in the environment is left alone,
so `GEMINI_API_KEY=... roslaunch ...` still wins over the file.

Standard library only, Python 3.8 (the robot's own Python).
"""

import logging
import os

log = logging.getLogger("env_file")


def load(path: str) -> int:
    """Put the file's names into os.environ. Returns how many it set (0 if there is no file)."""
    if not os.path.isfile(path):
        log.warning("no %s: the keys must come from the environment instead", path)
        return 0
    loaded = []
    with open(path) as lines:
        for number, line in enumerate(lines, 1):
            name, value = _parse(line)
            if name is None:
                if line.strip() and not line.lstrip().startswith("#"):
                    log.warning("%s line %d is not NAME=value, ignored", path, number)
                continue
            if os.environ.get(name):
                continue  # set in the environment already: that one wins
            os.environ[name] = value
            loaded.append(name)
    log.info("%s: loaded %s", path, ", ".join(loaded) if loaded else "nothing new")
    return len(loaded)


def _parse(line: str):
    """(name, value) of one line, or (None, None) for a comment, a blank or a broken line."""
    line = line.strip()
    if not line or line.startswith("#") or "=" not in line:
        return None, None
    name, _, value = line.partition("=")
    name = name.strip()
    if name.startswith("export "):
        name = name[len("export ") :].strip()
    if not name:
        return None, None
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1]
    return name, value
