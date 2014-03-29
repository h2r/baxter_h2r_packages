cmake_minimum_required(VERSION 2.8.3)

project(meldon_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  genmsg
  OpenCV
  eigen
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
  FILES
  RecognizedObject.msg
  RecognizedObjectArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/src/kdes/KernelDescriptors_CPU/Include
	${PROJECT_SOURCE_DIR}/src/kdes/KernelDescriptors_CPU
)

add_executable(talker src/talker.cpp)

target_link_libraries(talker
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PROJECT_SOURCE_DIR}/src/kdes/dependencies/matio/src/.libs/libmatio.so
  /usr/lib/libhdf5.so
)