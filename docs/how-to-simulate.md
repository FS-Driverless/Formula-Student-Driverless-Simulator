# Let's get ready to run the simulation
*Interested in building the project from source and contributing to the simulator? Check out the [development guide](how-to-develop.md)*

To run the simulation smoothly you need quite a fast windows computer with a modern videocard.
Minimally you will need:
* 8 core 2.3Ghz CPU
* 12 GB memory
* 30GB free SSD storage
* Recent NVidia card with Vulkan support and 3 GB of memory.

To check the video card drivers, run `vulkaninfo`. It should output a bunch of lines without errors.

## Launching the simulator
To launch the simulator, go to [releases](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/releases) and download the latest one.

Before you start, you have to create the folder and file `Formula-Student-Driverless-Simulator/settings.json` in your home directory.
This file contains the sensor configuration of the car.
You should create a folder in your **home directory** called `Formula-Student-Driverless-Simulator`. 
Inside the folder `Formula-Student-Driverless-Simulator` create a file called `settings.json` and copy-paste the contents of the settings.json file at the root of this repository inside.
This should get you started with the default sensor configuration, feel free to try your own custom sensor suite.
Note that the naming of the sensors will be reflected in the topic names as elaborated [here](ros-bridge.md).

Now launch FSDS.exe and a window with a car should popup!
Try to drive the car around by using the arrowkeys.

## Launching the ROS bridge
To connect your autonomous ROS system, you have to clone this repository and run the `fsds_ros_bridge` ROS node.

Most likely, your autonomous system is running on Ubuntu and has already ROS Melodic installed.
If this is not the case, read the relevant install instructions in the [get-ready-to-develop](get-ready-to-develop.md) guide that will help you get the requried software installed.

Ready? Lets clone the repo **into your home directory**:
```
git clone git@github.com:FS-Online/Formula-Student-Driverless-Simulator.git
```
The repository will be placed in `~/Formula-Student-Driverless-Simulator`.
If this folder already exists as a result of the previous step, you can get your settings.json out, delete the folder (or rename it to something like UEProject or UESim) and after cloning the repo place the settings.json file back (this time inside the root of the repository where the default settings.json lives).

Read about how to build the ROS workspace [here](building-ros.md).

Then, launch the [ros bridge](ros-bridge.md).