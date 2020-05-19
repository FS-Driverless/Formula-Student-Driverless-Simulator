cmake_minimum_required(VERSION 3.10.0)
project(examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  fsds_ros_bridge
)
catkin_package(
  CATKIN_DEPENDS rospy fsds_ros_bridge
)
