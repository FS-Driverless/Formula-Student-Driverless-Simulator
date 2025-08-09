cmake_minimum_required(VERSION 3.0.2)
project(formula_autonomous_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
  fs_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# No custom messages or services defined for this package
# generate_messages() removed since no message files are defined

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
  fs_msgs
  pcl_conversions
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
file(GLOB_RECURSE CPP_SOURCES "cpp/**/*.c" "cpp/**/*.cpp")

add_executable(formula_autonomous_system 
  ${CPP_SOURCES}
)

## Add cmake target dependencies of the executables
add_dependencies(formula_autonomous_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executables against
target_link_libraries(formula_autonomous_system
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  python/formula_autonomous_system.py
  python/formula_autonomous_system_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autonomous_dev_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
