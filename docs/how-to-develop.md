# How to develop this project

This guide describes how to work on this project.
Before you continue reading, make sure you have finished the [get-ready-to-develop](get-ready-to-develop.md) guide.

As you can read in the [system overview](system-overview.md) this system consists of multiple components.


To develop any of these components you have to run Unreal Engine with the AirSim plugin.
So we need to compile the AirSim plugin.

## Compile the AirSim plugin and launch Unreal Engine
The AirSim plugin sourcode is made up of AirLib (/AirSim/AirLib) and the plugin code (/UE4Project/Plugins/AirSim/Source).
AirLib is also used by the ros bridge.

First build the AirLib code. Open the _Developer Command Prompt for VS 2019_, go to `Driverless-Competition-Simulator/AirSim` and run
```
build.cmd
```
The first time this takes quite a while. Go walk around a bit, maybe start playing [factoryidle](https://factoryidle.com/). 

After it is finished, launch unreal engine and open the project file `Driverless-Competition-Simulator/UE4Project/FSOnline.uproject`

It might show an error like 'This project was made with a different version of the Unreal Engine'. In that case select `more options` and `skip conversion`.

When asked to rebuild the 'Blocks' and 'AirSim' modules, choose 'Yes'.
This is the step where the plugin part of AirSim is compiled.

After some time Unreal Engine will start and you can launch the game.

If you make changes to the plugin code you only have to recompile the plugin. 
This can be done from within the Unreal Editor. go to to `Window` -> `Developer tools` -> `Modules`. 
Search for `AirSim` and click `Recompile`.

If you make changes to AirLib you have to run `build.cmd` again.


## Ros development

To set up the ROS workspace, cd into the `ros` folder and run
```
catkin init
catkin build
```
Now you can [run the ros bridge](ros-bridge.md).
