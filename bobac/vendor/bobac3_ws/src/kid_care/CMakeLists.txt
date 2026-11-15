cmake_minimum_required(VERSION 3.0.2)
project(kid_care)

find_package(catkin REQUIRED)

catkin_package()

# The two nodes plus the care flow modules they import: all in one folder, so the imports
# work in the install space too.
catkin_install_python(PROGRAMS
  scripts/kid_care_node.py
  scripts/kid_care_start.py
  scripts/care_server.py
  scripts/care.py
  scripts/caregiver.py
  scripts/ntfy_channel.py
  scripts/speech.py
  scripts/listen.py
  scripts/marionette.py
  scripts/agent_process.py
  scripts/env_file.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY agent/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/agent)
