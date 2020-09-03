# Local setup

If you are planning to use 2 PC's, 1 Windows machine running the simulator and another machine running Linux, there are some things you should know.

## Python autonomous system

If your autonomous system is written in python and you only have 1 option at the moment to run the simulator with 2 local machines.
This is by running the `fsds-bridge` on the Linux machine. This means that you will not be able to ensure low latency to and from your Windows machine running the simulator.
But running the simulator will be exactly the same as in [the getting started page](getting-started.md#running-the-ros-bridge).

## C++ autonomous system
If your system is written in C++, you have 2 options:
- Running the `fsds-bridge` on your Linux machine &rarr; more latency between ROS bridge and simulator
- Running the `fsds-bridge` on your Windows machine in WSL &rarr; Running the `fsds/controlCommand` topic in UDP mode, might lose some packets

If you are using WSL 2 and you manage to get ROS to work with 2 machines, please help us understand how by writing a comment on [this](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/227) issue.

### Network setup
If you have never run ROS with multiple machines, follow [this tutorial](http://wiki.ros.org/ROS/NetworkSetup) and [this one](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) on the ROS Wiki.

If everything is setup correctly you should be able to run the bridge with the `UDP_control` argument. So you would launch the bridge as follows:
```
roslaunch fsds_ros_bridge fsds_ros_bridge.launch UDP_control:=true
```
The changes have been tested on WSL 1 (Ubuntu 18.04), second machine (Ubuntu 18.04) with ROS melodic on both machines.