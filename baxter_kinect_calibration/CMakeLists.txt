cmake_minimum_required(VERSION 2.8.3)
project(baxter_kinect_calibration)

# setup
set(MSG_DEPS
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs)

find_package(catkin REQUIRED
        cmake_modules
        eigen_conversions
        cv_bridge
        image_geometry
        image_transport
        camera_info_manager
        pcl_conversions
        pcl_ros
        roscpp
        tf
        ar_track_alvar
        ar_track_alvar_msgs
        genmsg
        tf2
        resource_retriever
        message_generation
        ${MSG_DEPS}
        dynamic_reconfigure)

find_package(Eigen REQUIRED)
#find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/Params.cfg)

#link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS
    cv_bridge
    eigen
    image_geometry
    image_transport
    camera_info_manager
    pcl_ros
    roscpp
    tf
    visualization_msgs
    std_msgs
    tf2
    message_runtime
    sensor_msgs
    geometry_msgs
    resource_retriever
    pcl_conversions
    dynamic_reconfigure)


include_directories(include
                    SYSTEM
 #                   ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                   )

set(GENCPP_DEPS std_msgs_gencpp sensor_msgs_gencpp geometry_msgs_gencpp visualization_msgs_gencpp)



add_executable(bundle_calibrate src/calibrate_kinect_alvar_bundle.cpp)
target_link_libraries(bundle_calibrate ar_track_alvar kinect_filtering medianFilter ${catkin_LIBRARIES})
add_dependencies(bundle_calibrate ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

set(CALIBRATION_TARGETS bundle_calibrate)

install(TARGETS ${CALIBRATION_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
