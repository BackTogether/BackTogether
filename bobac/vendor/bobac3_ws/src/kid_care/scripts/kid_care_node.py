#!/usr/bin/env python3
"""The care flow as a launch node: load the hidden .env, then run care_server.py.

care_server.py needs no ROS, so this only reads the keys from the hidden file the launch points
at (the launch's `env_file` argument) and hands the rest of the arguments over. roslaunch adds
its own `__name:=` / `__log:=` arguments; they are dropped here.

Started by kid_care.launch; by hand it is the same as running care_server.py:

    rosrun kid_care kid_care_node.py --env-file ~/kid_care/.env --port 8090
"""

import logging
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # the care flow modules live next to this file

import env_file  # noqa: E402

ENV_FILE_FLAG = "--env-file"
DEFAULT_ENV_FILE = HERE.parent / ".env"


def take_env_file(arguments):
    """The value of --env-file, removed from `arguments`: it is ours, not care_server's."""
    if ENV_FILE_FLAG not in arguments:
        return str(DEFAULT_ENV_FILE)
    at = arguments.index(ENV_FILE_FLAG)
    if at + 1 >= len(arguments):
        sys.exit("kid_care_node: " + ENV_FILE_FLAG + " needs a path")
    del arguments[at]
    return arguments.pop(at)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    arguments = [a for a in sys.argv[1:] if not a.startswith("__")]
    env_file.load(take_env_file(arguments))
    # after the keys are in the environment: care_server reads some of them while it loads
    import care_server

    sys.argv = [sys.argv[0]] + arguments
    care_server.main()


if __name__ == "__main__":
    main()
