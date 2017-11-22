cmake_minimum_required(VERSION 2.8.3)
project(uchile_battery_sensor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  rqt_robot_dashboard
  sensor_msgs
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
)
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# TODO: que instalar?
install(
  PROGRAMS
    scripts/notebook_battery_pub.py
    scripts/uchile_battery_sensor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config images)
 install(DIRECTORY ${dir}/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
