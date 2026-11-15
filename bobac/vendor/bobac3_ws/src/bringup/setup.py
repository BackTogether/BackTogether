#!/usr/bin/env python3
# Makes src/bringup/ importable from the nodes in scripts/ (catkin_python_setup in CMakeLists).
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=["bringup"], package_dir={"": "src"}))
