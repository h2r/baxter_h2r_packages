cmake_minimum_required(VERSION 2.8.3)
project(baxter_grasps_server)

find_package(catkin REQUIRED moveit_msgs)

catkin_python_setup()

add_service_files(
   FILES
   GraspService.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   moveit_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_grasps_server
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
