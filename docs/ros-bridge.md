# FSDS ROS bridge

![In action](images/fsds_ros_bridge.png)

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
- `/fsds/gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)   
This the current GPS coordinates of the drone in airsim published at 10hz
[Read all about the gps simulation model here.](gps.md)
Data is in the `fsds/FSCar` frame.

- `/fsds/imu` [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)   
Velocity, orientation and acceleratoin information at 250hz.
[Read all about the IMU model here.](imu.md)
Data is in the `fsds/FSCar` frame.

- `/fsds/odom` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
Ground truth car position and orientation in NED frame. THIS WILL NOT BE STREAMED DURING COMPETITION.

- `/fsds/CAMERA_NAME/IMAGE_TYPE` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
One of this topic type will exist for every camera specified in the `settings.json` file.
On this topic, camera frames are published. The format will be bgra8. 
`CAMERA_NAME` will be replaced by the corresponding in the `Cameras` object in the `settings.json` file.
`IMAGE_TYPE` is determand by the `SensorType` field. 
When choosing 0, it will be 'Scene'.

- `/fsds/CAMERA_NAME/IMAGE_TYPE/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)   
This topic publishes metadata about the related camera.
For every frame sent on `/fsds/CAMERA_NAME` 1 message will be sent on this topic.

- `/fsds/lidar/LIDARNAME` [sensor_msgs/PointCloud](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html)   
Publishes the lidar points for each lidar sensor.
All points are in the `fsds/LIDARNAME` frame.
Transformations between the `fsds/LIDARNAME` and `fsds/FSCar` frame are being published regularly.

- `/fsds/lidar/LIDARNAME` [sensor_msgs/PointCloud](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html).   
  Publishes the lidar points for each lidar sensor.

- `/fsds/signal/go` [fsds_ros_bridge/GoSignal](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/msg/GoSignal.msg)   
GO signal that is sent every second by the ROS bridge.
The car is only allowed to drive once this message has been received. 
If no GO signal is received for more than 4 seconds, the AS can assume that `fsds_ros_bridge` has been shut down.
This message also includes the mission type and track.
More info about signal topics can be found in the [integration handbook](integration-handbook.md)

- `/fsds/signal/finished` [fsds_ros_bridge/FinishedSignal](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/msg/FinishedSignal.msg)   
Finished signal that is sent by the AS to stop the mission. More info about signal topics can be found in the [integration handbook](integration-handbook.md)

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)   
See 'Coordinate frames and transforms'



## Subscribers
- `/fsds/control_command` [fsds_ros_bridge/ControlCommand](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/msg/ControlCommand.msg)   
This message includes the dimensionless values throttle, steering and brake. 
Throttle and brake range from 0 to 1.
For steering `-1` steers full to the left and `+1` steers full to the right.
The contents of this message fill the essential parts of the `msr::airlib::CarApiBase::CarControl` struct. 
This is the only way to control the car when the airsim ROS client is connected (keyboard will no longer work!).

- `/fsds/signal/finished` [fsds_ros_bridge/FinishedSignal](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/msg/FinishedSignal.msg)   
Finished signal that is sent by the AS to stop the mission.
The ros bridge will forward the signal to the operator which in turn will stop the ros bridge and finish the run.

## Services

- `/fsds/reset` [fsds_ros_bridge/Reset](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/srv/Reset.srv)   
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

- `/fsds/ros_bridge/update_airsim_img_response_every_n_sec` [double]   
  Set in: `$(fsds_ros_bridge)/launch/fsds_ros_bridge.launch`   
  Default: 0.01 seconds.   
  Timer callback frequency for receiving images from all cameras in airsim.
  The speed will depend on number of images requested and their resolution.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

## Visualization
This package contains some useful launch and config files which will help you in visualizing the data being streamed through the above topics.

To open Rviz with [this](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/config/rviz/default.rviz) configuration file, run `roslaunch fsds_ros_bridge rviz.launch`.

To open Multiplot with [this](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/ros/src/fsds_ros_bridge/config/multiplot/multiplot.xml) configuration file, run `roslaunch fsds_ros_bridge plot.launch`

## Monitoring
Performance monitoring of the ROS Bridge is described [here](statistics.md)