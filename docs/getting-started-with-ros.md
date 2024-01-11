# Connecting to the simulator with ROS

You can use the ROS bridge to connect the simulator to ROS1 and ROS2.
The ROS bridge will publish the sensordata from the simulator into ROS topics.
Your autonomous system will be able to publish car-control messages which the ROS bridge will send to the simulator.

The ROS bridge works on Ubuntu or in WSL in windows (but it is harder to set up).
If you have the simulator running on windows we reccommend Windows Subsystem for Linux.
This offers a virtual Ubuntu machine within Windows.
You can read [here how to install it](software-install-instructions.md). 
While you are at it you might also want to [add GUI support](software-install-instructions.md) so you can run rviz and rqt_plot from within WSL.

## Requirements

The ROS bridge requires [ROS Melodic/Noetic/Galactic to be installed](software-install-instructions.md), as well as the following dependencies:
```
sudo apt-get install ros-melodic-tf2-geometry-msgs python-catkin-tools ros-melodic-rqt-multiplot ros-melodic-joy ros-melodic-cv-bridge ros-melodic-image-transport libyaml-cpp-dev libcurl4-openssl-dev
```


## Cloning the repository

**Before you clone, make sure you have git lfs installed!**

Ready? Lets clone the repo into your home directory:
```
git clone git@github.com:FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

If you haven't setup your ssh keys, you can clone using https by running the following command:
```
git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

**THE REPO HAS TO BE CLONED IN THE HOME DIRECTORY!**. So the repo location should be `$HOME/Formula-Student-Driverless-Simulator`.
Why you ask? Because we couldn't get relative paths in the C++ code to work so now we have hard-coded some paths to the home directory.
I know yes it is ugly but it works. If you are bothered by it I would welcome you to open a pr with a fix.

If this folder already exists as a result of any previous step, move the existing folder out of the way and merge the content afterwards.

If you are on Windows and cloned this repository in a Windows directory, go into the cloned repo and run `git config core.fileMode false` to ignore file mode changes. 
If you want to share the the cloned directory with the Ubuntu WSL system, create a symlink within WSL like so:
```
ln -s /mnt/c/Users/developer/Formula-Student-Driverless-Simulator ~/Formula-Student-Driverless-Simulator
```

Now, checkout the version equal to the simulator. 
If you are running for example simulator packaged version v2.2.0, run `git checkout tags/v2.2.0` to get the ROS brige to the same version

## Preparing AirLib

AirLib is the shared code between the ROS wrapper and the AirSim Unreal Engine plugin.
We need to stage the source before we can compile it together with the wrapper.

If you are working in a WSL shared folder where previously build.cmd was ran, you can skip this step.

Open an Ubuntu terminal and run `AirSim/setup.sh`.
This will download the nessesary libraries required to compile AirLib.
You will only need to run this once.

Everything setup.sh does is also included in build.cmd.

## Building the workspace

```
cd ros
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release #(Optional)
catkin build
```

## Launching the ros bridge

The ROS bridge consists of a few nodes to achieve the highest performance and keep the codebase clean.
Everything can be launched using the `fsds_ros_bridge.launch` launchfile.
```
cd ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```

The ROS bridge will read the settings from `~/Formula-Student-Driverless-Simulator/settings.json`.
Make sure this is the same configuration file as the simulator uses.

[Read all about configuring the ROS bridge here.](ros-bridge.md)

## Connecting your autonomous system

The ROS bridge of this simulator had to make use of several custom msgs (for control commands, the groundtruth track, etc). 
These messages are defined in a ROS package called `fs_msgs` which is located in a separate, light [repository](https://github.com/FS-Driverless/fs_msgs). 
To implement publishers and subscibers for these messages types in your autonomous pipeline, you will have to add the `fs_msgs` repository as a submodule in your codebase (inside de `src` directory of an existing **catkin workspace** as done in this repository) or clone it and build it somewhere else in your system.

Now, all that is left to do is subscribe to the following topics to receive sensordata

- `/fsds/gps`
- `/fsds/imu`
- `/fsds/camera/CAMERA_NAME`
- `/fsds/camera/CAMERA_NAME/camera_info`
- `/fsds/lidar/LIDAR_NAME`
- `/fsds/testing_only/odom`
- `/fsds/testing_only/track`
- `/fsds/testing_only/extra_info`

and publish to the following topic `/fsds/control_command` to publish the vehicle control setpoints.

## Multiple computers
If you have 2 computer, you can run the simulator and your autonomous system each on their own computer.
But where does the ROS-bridge run? You have 2 options:

1. Run the ROS bridge on the same computer as your autonomous system.
   The ROS bridge will connect to the simulator using a TCP connection to send control commands and receive sensor data.
   The ROS bridge will use local ROS topics to communicate with the autonomous system.
   Use the `host` argument in the `fsds_ros_bridge.launch` file to tell the ROS bridge where the simulator is at.
   Ensure firewall rules allow the ROS bridge to connect to the simulator on port 41451.

2. Run the ROS bridge on the same computer as the simulator.
   Your autonomous system would use ROS multi-computer networking to publish/subscribe to FSDS topics.
   Follow [this tutorial](http://wiki.ros.org/ROS/NetworkSetup) and [this one](http://wiki.ros.org/ROS/Tutorials/ MultipleMachines) on the ROS Wiki to learn how to do this.

If you have never worked with a multi-computer ROS networking before, option 1 is probably the way to go.

If you are running the simulator on Windows, option 1 is the easiest as well.
You can run the ROS bridge within WSL and use option 2 but there are some constraints, see below.

## Notes on running the ROS bridge in WSL.

It is possible to run the ROS bridge in Windows Subsystem Linux (WSL).
However, when using WSL with a multi-computer ROS setup, [things get weird](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/227).
The problem is that everything crashes when you run the ROS bridge in WSL and try to send control commands via ROS from a different computer on the network.
To work around this problem, you should know:

**The only way to reliably have a ROS bridge in WSL receive control commands from another computer is to send these messages using UDP from a C++ node.**

To enable UDP on the control setpoint topic, set the `UDP_control` argument like so:

```
roslaunch fsds_ros_bridge fsds_ros_bridge.launch UDP_control:=true
```

UDP is only supported in roscpp. If you are using a Python node to send controll commands, UDP won't help a thing.

This setup has been tested on WSL 1 (Ubuntu 18.04), second machine (Ubuntu 18.04) with ROS melodic on both machines.

If you are using WSL 2 and you manage to get ROS to work with 2 machines, please help us understand how by writing a comment on [this](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/227) issue.
