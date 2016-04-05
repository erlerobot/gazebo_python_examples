cmake_minimum_required(VERSION 2.8.3)
project(erle_rover_followline)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  mavros
  mav_msgs
 )

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rospy
    mavros
    mav_msgs
)