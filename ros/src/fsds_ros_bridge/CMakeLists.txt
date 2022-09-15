cmake_minimum_required(VERSION 3.10.0)
project(fsds_ros_bridge)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  fs_msgs
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${Boost_INCLUDE_DIRS}
)

add_library(airsim_ros src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros ${catkin_LIBRARIES} yaml-cpp AirLib)

add_executable(fsds_ros_bridge src/fsds_ros_bridge.cpp)
target_link_libraries(fsds_ros_bridge airsim_ros ${catkin_LIBRARIES} AirLib)

add_executable(fsds_ros_bridge_camera src/fsds_ros_bridge_camera.cpp)
target_link_libraries(fsds_ros_bridge_camera ${catkin_LIBRARIES} AirLib)

find_package( PkgConfig REQUIRED)
pkg_check_modules( libcurl REQUIRED libcurl )
target_link_libraries(fsds_ros_bridge ${libcurl_LIBRARIES})


install(TARGETS 
    #list of shared libraries
    airsim_ros
    # pd_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)