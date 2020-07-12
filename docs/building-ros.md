# Building the ROS workspace
This guide describes how to use the ROS workspace on an Ubuntu machine.
It also works on Windows Subsystem for Linux 1 (WSL1)

If you do not have ROS Melodic installed, read the relevant instructions in the [get-ready-to-develop](get-ready-to-develop.md) guide.

Before we can build the workspace, install the workspace c++ library dependencies:
```
sudo apt-get install ros-melodic-tf2-geometry-msgs python-catkin-tools ros-melodic-rqt-multiplot ros-melodic-joy ros-melodic-cv-bridge ros-melodic-image-transport libyaml-cpp-dev libcurl4-openssl-dev
```

The first time when compiling this on a linux machine you need to run AirSim/setup.sh.
This will download the nessesary libraries required to compile airlib.
You will only need to run this once.
Everything setup.sh does is also included in build.cmd. 
So if you ran build.cmd in a WSL shared folder, you don't need setup.sh

- Build ROS package
```
cd ros
catkin init
catkin build
```