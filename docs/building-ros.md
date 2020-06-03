# Building the ROS workspace
This guide describes how to use the ROS workspace on an Ubuntu machine.
It also works on Windows Subsystem for Linux 1 (WSL1)

If you do not have ROS Melodic installed, read the relevant instructions in the [get-ready-to-develop](get-ready-to-develop.md) guide.

Before we can build the workspace, install the workspace c++ library dependencies:
```
sudo apt-get install ROS-melodic-tf2-geometry-msgs python-catkin-tools ROS-melodic-rqt-multiplot ROS-melodic-joy ROS-melodic-cv-bridge ROS-melodic-image-transport libyaml-cpp-dev
```

Then:

- Build AirSim shared c++ code. From the root of this repository, run:
```
cd AirSim
./setup.sh
./build.sh
```

- Build ROS package
```
cd ros
catkin build
```

If your default GCC isn't 8 or greater (check using `gcc --version`), then compilation will fail. In that case, use `gcc-8` explicitly as follows:

```
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```