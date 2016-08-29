cmake_minimum_required(VERSION 2.8.3)
project(erle_imu_visualizer)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS sensor_msgs rospy)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS sensor_msgs)

catkin_install_python(PROGRAMS
  scripts/imu_visu.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
