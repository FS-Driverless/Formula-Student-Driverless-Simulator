# Let's get ready to run the simulation

To run the simulation smootly you need quite a fast windows computer with a modern videocard.
Minimally you will need:
* 8 core 2.3Ghz CPU
* 12 GB memory
* 30GB free SSD storage
* Recent NVidia card with Vulkan support and 3 GB of memory.

To check the video card drivers, run `vulkaninfo`. It should output a bunch of lines without errors.

## Launching the simulator
To launch the simulator, go to [releases](https://github.com/FS-Online/Driverless-Competition-Simulator/releases) and download the latest one.

//todo: figure out where to configure the settings.json

Now launch the ????.exe and a window with a car should popup!
Try to drive the car around by using the arrowkeys.

## Launching the ros bridge
To connect you autonomous ros system, you have to clone this repository and run the `fsds_ros_bridge` ros node.

Most likely, your autonomous system is running on Ubuntu and has already Ros Melodic installed.
If this is not the case, read the relevant install instructions in the [get-ready-to-develop](get-ready-to-develop.md) guide.

Ready? Lets clone the repo **into your home directory**:
```
git clone git@github.com:FS-Online/Driverless-Competition-Simulator.git
```

Read about how to build the ros workspace [here](building-ros.md).

Then, launch the [ros bridge](ros-bridge.md).