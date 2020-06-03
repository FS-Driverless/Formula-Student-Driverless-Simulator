# Autonomous System Integration Handbook
This page describes how to integrate you autonomous system (AS) to the Formula Student Driverless Simulator (FSDS).
The rules and procedures set out in this document will be used during the FSOnline competition.

## High level overview
Your AS is expected to continuously run a ros master.
The simulator will launch a node ([fsds_ros_bridge](ros-bridge.md)) that connects to your ros system.
This node will publish sensordata and listen for vehicle setpoints.
When the simulation is done, the node is closed and your AS will no longer be able to interact with the simulator.
[A more in depth explanation of the fsds system can be found here.](system-overview.md)

Integrating your autonomous system with a simulator is a matter of subscribing and publishing to topics and acting accordingly.

## System flow and Signals

Initially, when your AS launches, no simulation is connected.
The AS must wait for a GO signal before acting.

### Staging
At some point your AS will be staged: The vehicle is placed at a staging line prior to the starting line, the `fsds_ros_bridge` node will connect to the ros system.
From this point onwards, the AS will receive sensordata and can controll the vehicle by publishing vehicle setpoints.
However, it shouldn't start driving the vehicle just yet!

### Starting 
Just like with a phisical FS event, the vehicle must only start driving after a GO signal is received.
The GO signal indicates that the AS must start the mision and that the vehicle should start driving.
This signal is also known as the 'green flag' signal or 'starting' signal, within this repo we will reference it as 'GO'.
Within the GO signal, the mission is added. 
Currently, `autocross` and `trackdrive` are the supported missions.

In autocross the vehicle has to complete a single lap on an unknown track.
On the trackdrive the vehicle has to finish 10 laps on a track it has previously seen.
During competition, on each track, every AS will do an autocross mission before an trackdrive mission.
It can occur that multiple autocross runs on different tracks take place before going to autocross.
It can also happen that multiple autocross runs take place on the same track.
For example, the AS might be requested to do:
1. autocross on track A
2. autocross on track B
3. trackdrive on track A
4. autocross on track C
5. autocross on track C (re-run)
6. trackdrive on track C

The AS must implement the following behaviour:
* When the AS is requested to do autocross on a track that it has seen before, it must delete any and all data it gathered during all previous runs on this track.
* When the AS is requested to do trackdrive on a track where it has done a trackdrive previously, it must delete any and all data it gathered during all previous trackdrive runs on this track. However, the data gatherd during the last autocross on this track shouldn't be deleted.

An exception to this rule is data recorded with the exclusive intent to analyze the AS's behaviour after the event.
This includes all files that the AS only writes to but does not read from.

To make the AS aware of which track it is driving, the GO signal includes a unique identifier of the upcomming track.

After the initial GO signal, the signal is continously re-sent 1 Hz to ensure it arrives at the team's AS.
The timestamp of all consecutive GO signals are equal to the first one.

### Finishing
There are two ways to conclude a run: finishing or stopping.

When the autonomous system feels it concluded it's run, it should send a FINISHED signal.
The FINISHED signal tells the simulator that the AS no longer wants to control the vehicle.
As such, the simulator will stop the `fsds_ros_bridge` and the AS will no longer receive sensor data or be able to control the vehicle.

When the official decides that the run is over it will stop the simulation.
See the rulebook for a description of when the official does so.
When the simulation is stopped the `fsds_ros_bridge` is stopped immediatly and and the AS will no longer receive sensor data or be able to control the vehicle.
The AS will not receive a signal that this happend.
To detect a stop, the AS should keep an eye on the GO signal.

The general rule is: If the AS did not receive a GO signal for 4 seconds the AS can assume the `fsds_ros_bridge` is stopped.
When this state is detected, the AS can reset itsself and prepare for a next mission.

## Sensor Suite
Every team can configure the sensors on their vehicle.
This configuration is called the sensor suite.
To ensure the simulation will perform as expected, the sensor suite has some restrictions.
Here you can read the requirements and restrictions that apply to every sensor.

### Camera

Every vehicle can have a maximum of 2 camera sensors. 
These camera camera(s) can be placed anywhere on the vehicle that would be allowed by FSG 2020 rules. 
The camera body dimensions are a 4x4x4 cm cube with mounting points at any side except the front facing side.

All camera sensors output uncompressed rgba8 images at 30 fps. 
You can choose the resolution of the camera(s). 
In total the camera’s can have 1232450 pixels. 
Every dimension (width or height) must be at least 240px and no greater than 1600px. 
The horizontal field of view (FoV) is configurable for each camera and must be at least 30 degrees and not be greater than 90 degrees. 
The vertical FoV will be automatically calculated using the following formula: `vertical FoV = image height / image width * horizontal FoV`.

The camera's auto exposure, motion blur and gamma settings will be equal for all teams.


### Lidar
A vehicle can have between 0 and 5 lidars.
The lidar(s) can be placed anywhere on the vehicle that would be allowed by FSG 2020 rules.
The body dimension of every lidar is a vertical cylinder, 8 cm heigh and 8 cm diameter with mounting points at the top and bottom.

A single lidar can have between 1 and 500 lasers. 
The lasers are stacked vertically and rotate on the horizontal plane. 
The lasers are distributed equally to cover the specified vertical field of view.

The vertical field of view is specified by choosing the upper and lower limit in degrees. 
The lower limit specifies the vertical angle between the horizontal plane of the lidar and the most bottom laser. 
The upper limit specifies the vertical angle between the horizontal plane of the lidar and most upper laser. 

The horizontal field of view of the lidar is specified with an upper and lower limit in degree as well. 
The lower limit specifies the counterclockwise angle on a top view from the direction the lidar is pointing towards. 
The upper limit specifies the clockwise angle on a top view from the direction the lidar is pointing towards. 

For every lidar the rotation speed (hz) and capture frequency (hz) must be chosen. 
The rotation speed specifies how fast the lasers spin and the capture frequency specifies how often a pointcloud is created.

While rotating, only lasers within the horizontal field of view are captured. 

> For example, a lidar with 190 degrees horizontal field of view, rotating at 10hz with a capture frequency of 20 hz will receive pointclouds covering anything from 10 to 180 degrees of the field of view.

There is no guarantee that the rotation speed and capture frequency stay in sync. 
You won’t be able to rely on synchronization of rotation speed and capture frequency. 
A lidar rotating at 5 hz would theoretically have rotated 50 times after 10 seconds but in reality this will be somewhere between 45 and 55 times. 

For every lidar you can specify the resolution: the total number of points collected if the lasers would do a 360 field of view sweep scan. 
This value is used to calculate the number of points in each laser and the spacing between the points. 

Every lidar capture is limited to collecting 10000 points. 
The maximum number of points collected during a capture is calculated by dividing the lidar’s resolution by the horizontal field of view.

> For example, a lidar with a 30 degrees horizontal field can have a maximum resolution of 120000.

The total number of points per second is limited to 100000 points.

> For example, a first lidar collects 10000 points per capture at 5 hz, a second lidar collects 8000 points per capture at 5 hz. 
  This is valid because in total they collect 90000 points per second.

### GPS
Every vehicle has 1 GPS, it is located at the center core of the vehicle.
This sensor cannot be removed or moved.

The GPS captures the position of the vehicle in the geodetic reference system, namely longitude [deg], latitude [deg], altitude [m].
More detailed technical information about the accuracy of the GPS can be found [here](gps.md).

### IMU
//todo: figure out how IMU works and describe it here.

### Sensor specification
Teams are expected to provide their sensor suite as a single AirSim settings.json file.
Most of the parameters in the settings.json file will be set by the officials to ensure fairness during competition.
You are allowed to configure the following subset of parameters within the boundries of above rules.

* Cameras
* * camera name
* * Width, Height
* * FOV_Degrees
* * X, Y, Z
* * Pitch, Roll, Yaw
* Lidars
* * NumberOfChannels
* * PointsPerSecond
* * RotationsPerSecond
* * HorizontalFOVStart
* * HorizontalFOVEnd
* * VerticalFOVUpper
* * VerticalFOVLower
* * X, Y, Z
* * Pitch, Roll, Yaw

The GPS and Lidar are configured equally for all teams according to the rules in the previous chapter.

We reccomand to copy the [settings.json in this repository](/UE4Project/Plugins/AirSim/Settings/settings.json) as a base and configure the cameras and lidar from there on.

## Launching the simulator
To run the simulation, read the [simulation guide](how-to-simulate.md).

## Ros integration
Communication between autonomous system and simulator will take place using ros topics.
Sensordata will be published by the [ros bridge](ros-bridge.md) and subscred on by the autonomous system.
The autonomous system will publish vehicle setpoints and the ros bridge will listen for those messages.
Transforms between sensors also are being published for usage by the autonomous system.

### Sensor topics
The following topics are made available:

- `/fsds/gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
GPS messages. [Read all about the gps model here](gps.md).

- `/fsds/camera/CAMERA_NAME` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
One of this topic type will exist for every camera specified in the `settings.json` file.
On this topic camera frames are published. The format will be bgra8. 
`CAMERA_NAME` will be replaced by the corresponding in the `Cameras` object in the `settings.json` file.

- `/fsds/camera/CAMERA_NAME/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
This topic publishes metadata about the related camera.
For every frame sent on `/fsds/CAMERA_NAME` 1 message will be sent on this topic.

//todo: imu

### Signal topics
//todo add when signals are implemented.

- `/fsds/signal/go`

- `/fsds/signal/finish`

### vehicle setpoints
Publishing on the following topic controlls the vehicle:

- `/fsds_ros_bridge/VEHICLE_NAME/control_command` [fsds_ros_bridge/ControlCommand](../ros/src/fsds_ros_bridge/msg/ControlCommand.msg)

This message inculdes throttle, steering and brake. 
Each value is dimensionless and goes from -1 to 1.
For steering `-1` steers full to the left and `+1` steers full to the right.


### Transforms
//todo: We have to figure out how these transforms are working.

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

## 3D vehicle model
//todo
This chapter will describe how to change the 3d model of the vehicle and how to provide the 3d model for usage during the competition. 
At this moment we have no idea how this works sooooo when we figure it out this will be filled in.

## Competition deployment
A few weeks before competition, each team will receive the ssh credentials to an Ubuntu google cloud instance.
This instance will have 8 vCPU cores, 30 gb memory (configuration n1-standard-8), 1 Nvidia Tesla T4 videocard and 100GB SSD disk.
The teams must install their autonomous system on this computer.

At competition, a separate google cloud instance will run the simulation software and the ros bridge. 
One by one the ros bridge will connect to the different teams computers an they will do their mission.

During the weeks leading up to the competition FSOnline will host multiple testing moments where the autonomous computers will be connected to the simulator and drive a few test laps.

More information about the procedure will be added later.
