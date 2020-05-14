# Joystick Controller Node 

If you are interested in driving the car around to gather training data without having to rely on your autonomous system, you can use an XBox controller to do so. 
Make sure you have [built the ros workspace](building-ros.md).

Simply run:

```
cd ros
source devel/setup.bash
roslaunch joystick joystick.launch
```

This node gets input from a joystick xbox controller (see http://wiki.ros.org/joy) and sends the values to the lowlevel controls (hardware) on topic /fsds_ros_bridge/FSCar/control_command 

* The right trigger (RT) controls the acceleration (gas) 
* The left trigger (LT) controls brake (negative acceleration).
* The x-axis of the left stick controlls the steering angle.
* Pressing and holding the B button enables boost mode.

When the joy driver initializes it sometimes sends very high values. 
This would cause the car to move unpredictably.
Therefore the controller is 'locked' when it is starts and when the controller is re-plugged in.
To unlock both triggers must be fully pressed (gas and brake) after which commands will be sent.

This node checks if the controller is plugged in by polling the linux device filename. 
If the controller is unplugged the file disapears and this node will start sending break.

When the controller is locked or the controller is unplugged the node will continuously sending break setpoints at 10hz.

During normal operation the maximum setpoints are limited quite a bit to make the car better controllable.
If you enable boost mode (press B) you get more power and the car will move faster.
To configure the vlaue mappings between xbox controller and car setpoints, go into `src/joystick.cpp` and change the value of the variables.

## HELP it doesn't work
Make sure you run this on a computer that has the xbox controller attached!
You can check if the controller is available by running the below command.
```
$ sudo ls /dev/input/js0
crwxrwxrwx 1 root 993 13, 0 Nov  8 14:43 /dev/input/js0
```

If you get permisison erros you have to give ros more permissions. Run
```
sudo chmod 777 /dev/input/js0
```

If the device is connected but not available at js0 it might be mapped to another device.
Find the correct device by running
```
sudo ls -al /dev/input/js*
```
When you find the correct device mapping, update the launchfile accordingly.

The message
```
[ERROR] Couldn't open joystick force feedback!
```
is normal. Nothing to worry about. It is a warning that always happens with wired xbox controllers.

## testing
To test this node on your computer just attach an xbox controller and run it as described above.
Now chek the /fsds_ros_bridge/FSCar/control_command topic and you should see values corresponding to your controller movements.
You can debug the input values from the joy driver by checking the `/joy` topic.

## Subscribers:
- `/joy` [sensor_msgs/Joy](https://github.com/microsoft/AirSim/tree/master/ros/src/fsds_ros_bridge/msg/GPSYaw.msg)   
  Listens to joystick input which is then mapped to the control command msg. The mapping should feel intuitive but in case something is unclear, it is described in detail [here](../Simulator/src/fsds_ros_bridge/src/joystick.cpp) 

## Publishers:
- `/fsds_ros_bridge/VEHICLE_NAME/control_command` [fsds_ros_bridge/ControlCommand](../Simulator/src/fsds_ros_bridge/msg/ControlCommand.msg) 

