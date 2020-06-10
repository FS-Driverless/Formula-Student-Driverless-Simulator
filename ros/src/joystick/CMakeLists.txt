cmake_minimum_required(VERSION 3.10.0)
project(joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  fs_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp joy fs_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joycontrol src/joystick.cpp)
target_link_libraries(joycontrol ${catkin_LIBRARIES})