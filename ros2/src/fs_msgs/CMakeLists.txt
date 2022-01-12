cmake_minimum_required(VERSION 2.8.3)
project(fs_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  ControlCommand.msg
  FinishedSignal.msg
  GoSignal.msg
  Track.msg
  Cone.msg
  ExtraInfo.msg
)

add_service_files(
  FILES
  Reset.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)

