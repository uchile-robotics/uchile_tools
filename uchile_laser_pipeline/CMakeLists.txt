cmake_minimum_required(VERSION 2.8.3)
project(uchile_laser_pipeline)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS 
  angles
  dynamic_reconfigure
  filters
  laser_filters
  laser_geometry
  message_filters
  pluginlib
  roscpp
  rostest
  sensor_msgs
  tf
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS signals system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# dynamic_reconfigure
generate_dynamic_reconfigure_options(cfg/SelfFilter.cfg)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uchile_laser_pipeline
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
)

##############################################################################
# Build
##############################################################################


# self filter
add_library(uchile_laser_pipeline src/laser_scan_filters.cpp src/self_filter.cpp)
add_dependencies(uchile_laser_pipeline ${PROJECT_NAME}_gencfg)
target_link_libraries(uchile_laser_pipeline ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# scan limiter
add_executable(laser_scan_limiter src/laser_scan_limiter.cpp)
target_link_libraries(laser_scan_limiter ${catkin_LIBRARIES})