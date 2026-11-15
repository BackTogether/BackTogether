cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/face_check.py
  scripts/splash.py
  scripts/child_goal_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/assets)
