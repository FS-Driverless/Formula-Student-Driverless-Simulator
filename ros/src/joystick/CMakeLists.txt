cmake_minimum_required(VERSION 3.10.0)
project(joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  fsds_ros_bridge
)
catkin_package(
  CATKIN_DEPENDS roscpp joy fsds_ros_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joycontrol src/joystick.cpp)
target_link_libraries(joycontrol ${catkin_LIBRARIES})