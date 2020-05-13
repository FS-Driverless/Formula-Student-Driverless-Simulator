# Airsim ROS wrapper (Car API)

A ROS wrapper over the AirSim C++ **Car** client library. This code is based on the [original AirSim ROS wrapper for the *Multirotor* API](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_interface) and provides an interface between AirSim + Unreal Engine and your ROS-based autonomous system. 

## Prerequisites

Before being able to test this ROS wrapper, you will have to follow the steps described [here](get-ready-to-develop.md).

##  Build
- Build AirSim. From the root of this repository, run:
```
cd AirSim
./setup.sh
./build.sh
```
- Build ROS package

```
cd Simulator
catkin build
```

If your default GCC isn't 8 or greater (check using `gcc --version`), then compilation will fail. In that case, use `gcc-8` explicitly as follows-

```
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```

## Running
```
source devel/setup.bash
roslaunch airsim_ros_interface airsim_node.launch
```

# Using AirSim ROS wrapper
The ROS wrapper is composed of two ROS nodes - the first is a wrapper over AirSim's car C++ client library, and the second is a joystick controller interface node.    
Let's look at the ROS API for both nodes: 

### AirSim ROS Wrapper Node
#### Publishers:
- `/airsim_node/origin_geo_point` [airsim_ros_interface/GPSYaw](../Simulator/src/airsim_ros_interface/msg/GPSYaw.msg)   
GPS coordinates corresponding to global NED frame. This is set in the airsim's [settings.json](https://microsoft.github.io/AirSim/docs/settings/) file under (located [here](../UE4Project/Plugins/AirSim/Settings/settings.json)) the `OriginGeopoint` key. 
  
- `/airsim_node/VEHICLE_NAME/global_gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)   
This the current GPS coordinates of the drone in airsim. 

- `/airsim_node/VEHICLE_NAME/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
Odometry in NED frame wrt starting point.  THIS WILL NOT BE STREAMED DURING COMPETITION.

- `/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

- `/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  RGB or float image depending on image type requested in [settings.json](../UE4Project/Plugins/AirSim/Settings/settings.json).

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

where `VEHICLE_NAME`, `CAMERA_NAME` and `IMAGE_TYPE` are extracted from [settings.json](../UE4Project/Plugins/AirSim/Settings/settings.json).

#### Subscribers: 
- `/airsim_node/VEHICLE_NAME/control_command` [airsim_ros_interface/ControlCommand](../Simulator/src/airsim_ros_interface/msg/ControlCommand.msg) 
The contents of this message fill the essential parts of the `msr::airlib::CarApiBase::CarControl` struct. This is the only way to control the car when the airsim ROS client is connected (keyboard will no longer work!).

#### Services:

- `/airsim_node/reset` [airsim_ros_interface/Reset](../Simulator/src/airsim_ros_interface/srv/Empty.html)
 Resets car to start location.

#### Parameters:
- `/airsim_node/update_airsim_control_every_n_sec` [double]   
  Set in: `$(airsim_ros_interface)/launch/airsim_node.launch`   
  Default: 0.01 seconds.    
  Timer callback frequency for updating drone odom and state from airsim, and sending in control commands.    
  The current RPClib interface to unreal engine maxes out at 50 Hz.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 

- `/airsim_node/update_airsim_img_response_every_n_sec` [double]   
  Set in: `$(airsim_ros_interface)/launch/airsim_node.launch`   
  Default: 0.01 seconds.    
  Timer callback frequency for receiving images from all cameras in airsim.    
  The speed will depend on number of images requested and their resolution.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 

### Joystick Controller Node 

If you are interested in driving the car around to gather training data without having to rely on your autonomous system, you can use an XBox controller to do so. Simply run:

```
source devel/setup.bash
roslaunch airsim_ros_interface joystick.launch
```

#### Subscribers:
- `/joy` [sensor_msgs/Joy](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_interface/msg/GPSYaw.msg)   
  Listens to joystick input which is then mapped to the control command msg. The mapping should feel intuitive but in case something is unclear, it is described in detail [here](../Simulator/src/airsim_ros_interface/src/joystick.cpp) 

#### Publishers:
- `/airsim_node/VEHICLE_NAME/control_command` [airsim_ros_interface/ControlCommand](../Simulator/src/airsim_ros_interface/msg/ControlCommand.msg) 

### Visualization
This package contains some useful launch and config files which will help you in visualizing the data being streamed through the above topics.

To open Rviz with [this](../Simulator/src/airsim_ros_interface/config/rviz/default.rviz) configuration file, run `roslaunch airsim_ros_interface rviz.launch`.

To open Multiplot with [this](../Simulator/src/airsim_ros_interface/config/multiplot/multiplot.xml) configuration file, run `roslaunch airsim_ros_interface plot.launch`
