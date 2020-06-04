# FSDS ROS bridge

![In action](images/fsds.png)

A ROS wrapper over the AirSim C++ **Car** client library. This code is based on the [original AirSim ROS wrapper for the *Multirotor* API](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_interface) and provides an interface between AirSim + Unreal Engine and your ros-based autonomous system. 

The fsds_ros_bridge is supposed to be launched pointing at the Autonomous System's ROS master so that it can publish and subscribe to topics within the autonomous system. 
Physically this node should run on the airsim simulation server (that is the one that also runs the Unreal) project.
The node connects to the AirSim plugin, periodically retrieves sensor data (images, lidar, imu, gps) and publishes it on ROS topics.
It listens for car setpoints on other another and forwards these to the AirSim plugin.

## Running
Make sure you have [built the ROS workspace](building-ros.md).

```
cd ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```

## Publishers
- `/fsds/VEHICLE_NAME/global_gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)   
This the current GPS coordinates of the drone in airsim. 

- `/fsds/VEHICLE_NAME/odom` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
Ground truth car position and orientation in NED frame. THIS WILL NOT BE STREAMED DURING COMPETITION.

- `/fsds/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

- `/fsds/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  RGB or float image depending on image type requested in [settings.json](../settings.json).

- `/fsds/VEHICLE_NAME/imu` [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)   
  See [imu.md](imu.md)

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

- `/fsds/imu` [sensor_msgs/Imu](https://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
IMU messages. [Read all about the IMU model here](imu.md).

where `VEHICLE_NAME`, `CAMERA_NAME` and `IMAGE_TYPE` are extracted from [settings.json](../settings.json).

## Subscribers
- `/fsds/VEHICLE_NAME/control_command` [fsds_ros_bridge/ControlCommand](../ros/src/fsds/msg/ControlCommand.msg) 
The contents of this message fill the essential parts of the `msr::airlib::CarApiBase::CarControl` struct. This is the only way to control the car when the airsim ROS client is connected (keyboard will no longer work!).

## Services

- `/fsds/reset` [fsds_ros_bridge/Reset](../ros/src/fsds/srv/Empty.html)
 Resets car to start location.

## Coordinate frames and transforms

The primary frame is the `fsds/FSCar` frame.
This frame centers the center of the car.
The center of the car is the Unreal Engine car pawn position, which in turn is also the center of gravity.
It is using the NED coordinate system.
The `fsds/FSCar/enu` frame is the same point, translated to the ENU system.
Read more about the differences between ENU and NED [here](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates).

The ros bridge regularly publishes static transforms between the `fsds/FSCar` frame and each of the cameras and lidars.
Naming of these frames is `fsds/SENSORNAME`.
For example, a lidar named `Example` will publish it's points in the `fsds/Example` frame.

Only static transforms within the vehicle are published.
Transforms to the ground truth are disabled because this would take away the challenge of the competition.

## Parameters
- `/fsds/update_gps_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 0.1 seconds (10hz).
  Timer callback frequency for updating and publishing the gps sensordata.
  This value must be equal or higher to the update frequency of the sensor configured in the settings.json

- `/fsds/update_imu_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 0.004 seconds (250hz).
  Timer callback frequency for updating and publishing the imu messages.
  This value must be equal or higher to the minimual sample rate of the sensor configured in the settings.json

- `/fsds/update_odom_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 0.004 seconds (250hz).
  Timer callback frequency for updating and publishing the odometry.

- `/fsds/publish_static_tf_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 1 seconds (1 hz).
  The frequency at which the static transforms are published.

- `/fsds/update_lidar_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 0.1 seconds (10 hz).
  The frequency at which the lidar is publshed.

- `/fsds/update_airsim_img_response_every_n_sec` [double]
  Set in: `$(fsds_ros_bridge)/launch/fsds.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all cameras in airsim.
  The speed will depend on number of images requested and their resolution.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

## Visualization
This package contains some useful launch and config files which will help you in visualizing the data being streamed through the above topics.

To open Rviz with [this](../ros/src/fsds/config/rviz/default.rviz) configuration file, run `roslaunch fsds_ros_bridge rviz.launch`.

To open Multiplot with [this](../ros/src/fsds/config/multiplot/multiplot.xml) configuration file, run `roslaunch fsds_ros_bridge plot.launch`

## Monitoring
Performance monitoring of the ROS Bridge is described [here](statistics.md)