cmake_minimum_required(VERSION 2.8.3)
project(uchile_safety)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  uchile_srvs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    sensor_msgs
    uchile_srvs
    visualization_msgs
)