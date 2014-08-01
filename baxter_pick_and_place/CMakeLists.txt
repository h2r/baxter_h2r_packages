cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  baxter_interface
  geometry_msgs
  moveit_commander
  moveit_msgs
  object_recognition_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 install(PROGRAMS
   scripts/grasp.py
   scripts/listen_grasp.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )