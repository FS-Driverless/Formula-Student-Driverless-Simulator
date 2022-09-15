cmake_minimum_required(VERSION 3.18)
project(fsds_ros2_bridge)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(
  include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
)

find_package( PkgConfig REQUIRED)
pkg_check_modules( libcurl REQUIRED libcurl )

ament_auto_add_library(airsim_ros src/airsim_ros_wrapper.cpp)
target_link_libraries(airsim_ros yaml-cpp AirLib)

ament_auto_add_executable(fsds_ros2_bridge src/fsds_ros2_bridge.cpp)
target_link_libraries(fsds_ros2_bridge airsim_ros AirLib ${libcurl_LIBRARIES})

ament_auto_add_executable(fsds_ros2_bridge_camera src/fsds_ros2_bridge_camera.cpp)
target_link_libraries(fsds_ros2_bridge_camera AirLib ${OpenCV_LIBS})

# install(TARGETS 
#     #list of shared libraries
#     airsim_ros
#     # pd_position_controller_simple
#     ARCHIVE DESTINATION lib/${PROJECT_NAME}/
#     LIBRARY DESTINATION lib/${PROJECT_NAME}/
# )

# install(FILES 
#   #list of necessary files (xml...)
#   README.md
#   # settings.json
#   # nodelet_plugins.xml
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_auto_package(INSTALL_TO_SHARE launch)

