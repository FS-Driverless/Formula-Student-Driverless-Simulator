# Connecting to the simulator with ROS

You can use the ROS bridge to connect the simulator to ROS.
The ROS bridge will publish the sensordata from the simulator into ROS topics.
Your autonomous system will be able to publish car-control messages which the ROS bridge will send to the simulator.

The ROS bridge only works on Ubuntu.
If you have the simulator running on windows we reccommend Windows Subsystem for Linux.
This offers a virtual Ubuntu machine within Windows.
You can read [here how to install it](software-install-instructions.md). 
While you are at it you might also want to [install Xming](software-install-instructions.md) so you can run rviz and rqt_plot from within WSL.

## Requirements

The ROS bridge requires [ROS Melodic to be installed](software-install-instructions.md), as well as the following dependencies:
```
sudo apt-get install ros-melodic-tf2-geometry-msgs python-catkin-tools ros-melodic-rqt-multiplot ros-melodic-joy ros-melodic-cv-bridge ros-melodic-image-transport libyaml-cpp-dev libcurl4-openssl-dev
```


## Cloning the repository

**Before you clone, make sure you have git lfs installed!**

Ready? Lets clone the repo into your home directory:
```
git clone git@github.com:FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

If you havn't setup your ssh keys, you can clone using https by running the following command:
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
If you are running for example simulator packaged version v2.0.0, run `git checkout tags/v2.0.0` to get the ROS brige to the same version

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

and publish to the following topic `/fsds/control_command` to publish the vehicle control setpoints.

## Multiple computers
The ROS bridge should preferable run on the same computer as the simulator to ensure low latency and high bandwidth.
However, it is possible to run the ROS bridge on a different computer than the simulator.
To get this to work you should use the `host` argument in the `fsds_ros_bridge.launch` file.
The ROS bridge will connect to the simulator on port 41451.