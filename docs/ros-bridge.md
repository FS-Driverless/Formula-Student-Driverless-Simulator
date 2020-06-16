# FSDS ROS bridge

![In action](images/fsds_ros_bridge.png)

A ROS wrapper over the AirSim C++ **Car** client library. This code is based on the [original AirSim ROS wrapper for the *Multirotor* API](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_interface) and provides an interface between AirSim + Unreal Engine and your ros-based autonomous system. 

The fsds_ros_bridge is supposed to be launched pointing at the Autonomous System's ROS master so that it can publish and subscribe to topics within the autonomous system. 
Physically this node should run on the airsim simulation server (that is the one that also runs the Unreal) project.
The node connects to the AirSim plugin, periodically retrieves sensor data (images, lidar, imu, gps) and publishes it on ROS topics.
It listens for car setpoints on other another and forwards these to the AirSim plugin.

## Running
Make sure you have [built the ROS workspace](building-ros.md).

The ros bridge consists of a different few nodes to achieve the highest performance and keep the codebase clean.
Everything can be launched using the `fsds_ros_bridge.launch` launchfile.
```
cd ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```

This launches the following nodes:
* `/fsds/ros_bridge` node responsible for IMU, GPS, lidar, vehicle setpoints and go/finish signals.
* `/fsds/camera/CAMERANAME` node is run for each camera configured in the `settings.json`. The nodes are launched using the `cameralauncher.py` script.

## Published topics


| Topic name | Description | Message | Rate (hz) |
|---|---|---|---|
| `/fsds/gps` | This the current GPS coordinates of the drone in airsim. Read all about the gps simulation model [here](gps.md). Data is in the `fsds/FSCar` frame. | [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) | 10 |
| `/fsds/imu` | Velocity, orientation and acceleration information. Read all about the IMU model [here](imu.md). Data is in the `fsds/FSCar` (enu) frame. | [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)  | 250 |
| `/fsds/testing_only/odom_enu` | Ground truth car position and orientation in ENU frame about the CoG of the car (`fsds/FSCar`).  The units are `m` for distance and `rad` for angles. *THIS WILL NOT BE STREAMED DURING COMPETITION.* | [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)  | 250 |
| `/fsds/testing_only/track` | Ground truth cone position and color with respect to the starting location of the car. Currently this only publishes the *initial position* of cones that are part of the track spline. Any cones placed manually in the world are not published here. Additionally, the track is published once and the message is latched (meaning it is always available for a newly created subscriber). *THIS WILL NOT BE STREAMED DURING COMPETITION.* | [fs_msgs/Track](https://github.com/FS-Online/fs_msgs/blob/master/msg/Track.msg)  | Latched |
| `/fsds/camera/CAMERA_NAME` | One of this topic type will exist for every camera specified in the `settings.json` file. On this topic, camera frames are published. The format will be bgra8. `CAMERA_NAME` will be replaced by the corresponding in the `Cameras` object in the `settings.json` file. `IMAGE_TYPE` is determand by the `SensorType` field. When choosing 0, it will be 'Scene'. | [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)  | ~18 |
| `/fsds/lidar/LIDARNAME` | Publishes the lidar points for each lidar sensor. All points are in the `fsds/LIDARNAME` frame. Transformations between the `fsds/LIDARNAME` and `fsds/FSCar` frame are being published regularly. More info on the lidar sensor can be found [here](integration-handbook.md) | [sensor_msgs/PointCloud](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html)  | 10 |
| `/fsds/signal/go` | GO signal that is sent every second by the ROS bridge.The car is only allowed to drive once this message has been received. If no GO signal is received for more than 4 seconds, the AS can assume that `fsds_ros_bridge` has been shut down. This message also includes the mission type and track. More info about signal topics can be found in the [integration handbook](integration-handbook.md) | [fs_msgs/GoSignal](https://github.com/FS-Online/fs_msgs/blob/master/msg/GoSignal.msg)  | 1 |
| `/tf_static` | See [Coordinate frames and transforms](#coordinate-frames-and-transforms) | [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)   | 1 |


## Subscribed topics

| Topic name | Description | Message |
|---|---|---|
| `/fsds/control_command` | This message includes the dimensionless values throttle, steering and brake. Throttle and brake range from 0 to 1. For steering `-1` steers full to the left and `+1` steers full to the right. The contents of this message fill the essential parts of the `msr::airlib::CarApiBase::CarControl` struct.  This is the only way to control the car when the airsim ROS client is connected (keyboard will no longer work!). | [fs_msgs/ControlCommand](https://github.com/FS-Online/fs_msgs/blob/master/msg/ControlCommand.msg) |
| `/fsds/signal/finished` | Finished signal that is sent by the AS to stop the mission. The ros bridge will forward the signal to the operator which in turn will stop the ros bridge and finish the run. | [fs_msgs/FinishedSignal](https://github.com/FS-Online/fs_msgs/blob/master/msg/FinishedSignal.msg) |


## Services

- `/fsds/reset` [fsds_ros_bridge/Reset](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/srv/Reset.srv)   
 Resets car to start location.

## Units

If a topic streams a standard ROS message (like [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)) then the units will be the recommended units in the message documentation. Custom messages (from the [fs_msgs](https://github.com/FS-Online/fs_msgs) package) use the units specified in the message documentation as well. If in doubt, interpret distances in meters, angles in radians and rates in m/s and rad/s, etc.

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
The position and orientation of a camera named `Test` will become available in the frame `/fsds/Test`.

Only static transforms within the vehicle are published.
Transforms to the ground truth are disabled because this would take away the challenge of the competition.

## Parameters
- `/fsds/ros_bridge/update_gps_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 0.1 seconds (10hz).   
  Timer callback frequency for updating and publishing the gps sensordata.
  This value must be equal or higher to the update frequency of the sensor configured in the settings.json

- `/fsds/ros_bridge/update_imu_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 0.004 seconds (250hz).   
  Timer callback frequency for updating and publishing the imu messages.   
  This value must be equal or higher to the minimual sample rate of the sensor configured in the settings.json

- `/fsds/ros_bridge/update_odom_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 0.004 seconds (250hz).   
  Timer callback frequency for updating and publishing the odometry.

- `/fsds/ros_bridge/publish_static_tf_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 1 seconds (1 hz).   
  The frequency at which the static transforms are published.

- `/fsds/ros_bridge/update_lidar_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 0.1 seconds (10 hz).   
  The frequency at which the lidar is publshed.

## Visualization
This package contains some useful launch and config files which will help you in visualizing the data being streamed through the above topics.

To open Rviz with [this](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/config/rviz/default.rviz) configuration file, run `roslaunch fsds_ros_bridge rviz.launch`.

To open Multiplot with [this](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/config/multiplot/multiplot.xml) configuration file, run `roslaunch fsds_ros_bridge plot.launch`

## Monitoring
Performance monitoring of the ROS Bridge is described [here](statistics.md)