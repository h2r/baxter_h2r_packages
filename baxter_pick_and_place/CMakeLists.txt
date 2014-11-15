cmake_minimum_required(VERSION 2.8.3)
project(baxter_pick_and_place)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
  moveit_core 
  tf
  moveit_msgs
  moveit_ros_planning 
  moveit_ros_planning_interface 
  baxter_core_msgs
  baxter_grasps_server
  baxter_props
  baxter_tools
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  roscpp
  cmake_modules
)


find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

catkin_python_setup()

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(marker_pick_place src/marker_pick_place.cpp)
target_link_libraries(marker_pick_place ${catkin_LIBRARIES})
add_definitions(${EIGEN_DEFINITIONS})

 install(PROGRAMS
   scripts/grasp.py
   scripts/listen_grasp.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )