cmake_minimum_required(VERSION 2.8.3)
project(uchile_fun)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  sound_play
  std_msgs
  std_srvs
  tf
  uchile_msgs
  uchile_srvs
)
find_package(OpenCV  REQUIRED)

catkin_python_setup()

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fpermissive")

###################################
## catkin specific configuration ##
###################################

add_service_files(
  FILES
  UploadImage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES uchile_fun
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    pcl_conversions
    roscpp
    roslib
    sensor_msgs
    std_msgs
    uchile_msgs
    uchile_srvs
  DEPENDS opencv2
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

#add_executable(play_video src/play_video.cpp)
#add_dependencies(play_video ${catkin_EXPORTED_TARGETS})
#target_link_libraries(play_video ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )

#add_executable(facemask src/facemask.cpp)
#add_dependencies(facemask ${catkin_EXPORTED_TARGETS})
#target_link_libraries(facemask ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )

#add_executable(videomask src/videomask.cpp)
#add_dependencies(videomask ${catkin_EXPORTED_TARGETS})
#target_link_libraries(videomask ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )
