# Getting started

When running this simulator there are two steps: running the simulator and connecting your autonomous system.
This page helps you run the simulation.
After you have finished this page you can go on and connect your autonomous system. 

To run the simulation smoothly you need quite a fast Windows computer with a modern videocard.
We highly recommend the following computer specs. 
You might be able to run with less power but everything will be slower.
For developing this project, you need quite a good computer because Unreal Engine is a heavy baby.

* 8 core 2.3Ghz CPU
* 12 GB memory
* 30GB free SSD storage (120GB when building the unreal project from source)
* Recent NVidia card with Vulkan support and 3 GB of memory. (You can check the video card drivers by running `vulkaninfo`). Different brand video cards might work but have not been tested.

If your computer does not suffice you can use a remote workstation on Google Cloud Platform.
Read [this tutorial](gcp-remote-workstation.md) on how to setup your virtual workstation.

The simulator will load settings from the file `Formula-Student-Driverless-Simulator/settings.json` in your **home directory**.
This file is required for the simulator to work and contains the sensor configuration of the car.
If you clone the repo you will already have this file in place.
If not, copy-paste the contents of the [settings.json file at the root of this repository](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/settings.json) into the `~/Formula-Student-Driverless-Simulator`.
This should get you started with the default sensor configuration, feel free to try your own custom sensor suite.

## From release binaries

The simulator is distributed as binaries on every release.
[At this moment only windows binaries are released. For now, if you are on Ubuntu you will have to run the simulator from the Unreal Engine Editor.](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/107)
During competition, the simulation will run on Windows because it offers a bit better performance and stability.

Go to [releases](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases) and download the latest one.
Unzip it to anywhere on your computer and launch FSDS.exe.
A window with a car should popup!
Try driving the car around using the arrowkeys.
If you get a black screen with some buttons, make sure the folder with the binary is in your user folder (Windows: `C:\Users\username\Formula-Student-Driverless-Simulator`, Linux: `~/Formula-Student-Driverless-Simulator`)

## From source using the Unreal Engine Editor
Instead of running the simulator from release binaries, you can compile it manually using unreal engine.
This is usefull if you want to get the latest changes or if you want to make changes to the maps or the simulation itself.
If you want to run the unreal engine project from source you will need [unreal engine and visual studio 2019](software-install-instructions.md).
On Ubuntu you can skip the visual studio 2019 part, but you still need Unreal Engine.

### 1. Get the repository

You can either download the repo using the big green download button on the [github page of this project](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator) or clone the repository. For cloning, checkout the documentation on this further down this page. Make sure you clone the repository in your **home directory**.

When downloading or cloning, by default you get the latest, unreleased version. This is probably not the version that you want. Make sure you select the version that you need! 

### 2. Compiling the AirSim plugin
The Unreal Engine project requires the AirSim plugin.
We have to compile this plugin first.
The AirSim plugin is made up of AirLib (/AirSim/AirLib) and the plugin code (/UE4Project/Plugins/AirSim/Source).
AirLib is the shared code between the ros wrapper and the AirSim Unreal Engine plugin.

First build the AirLib code.

On Windows, open the _Developer Command Prompt for VS 2019_, go to `Formula-Student-Driverless-Simulator/AirSim` and run
```
build.cmd
```

On Ubuntu, go to folder `AirSim` and run `setup.sh` and `build.sh`.

> So what does build.cmd or setup.sh+build.sh do? 
  It downloads any nessesary libraries and compiles AirLib.
  After compilation it places the files in the UE4Project so that these files can be used durcing compilation of the plugin.

The first time this takes quite a while. Go walk around a bit, maybe start playing [factoryidle](https://factoryidle.com/). 

### 3. Working with the Unreal Engine project

Launch Unreal Engine, press Browse and open the FSDS project in `~/Driverless-Competition-Simulator/UE4Project/FSOnline.uproject`. 
The project should now open correctly. 
If it does not, make sure of the following:

 * you have cloned the repository inside your home folder (~/) 
 * you have cloned with LFS enabed. If not, run `git lfs install` and `git lfs pull` to download the large files.
 * within `~/Driverless-Competition-Simulator/AirSim/`, you have run `build.cmd` on Windows and `./setup.sh && ./build.sh` on Ubuntu.

On Ubuntu, we recommend adding the following alias to your ~/.bashrc to speed up the process of opening the repository in the future:
`alias ue='~/UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject'`

It might show an error like 'This project was made with a different version of the Unreal Engine'. In that case select `more options` and `skip conversion`.

When asked to rebuild the 'Blocks' and 'AirSim' modules, choose 'Yes'.
This is the step where the plugin part of AirSim is compiled.

After some time Unreal Engine will start and you can launch the game. 
Run the game in standalone mode or or selected viewport mode, simulate and eject mode do not support camera's.

If you make changes to AirLib you have to run `build.cmd` again.

If you make changes to the plugin code or AirLib, you only have to recompile the plugin.
This can be done from within the Unreal Editor. go to to `Window` -> `Developer tools` -> `Modules`.
Search for `AirSim` and click `Recompile`.

### 4. Launching the game

To run the game, click the big Play button.
If you want to run it like it would run when packaged, choose 'Run as standalone game'.

## Next steps
Now you are ready to connect your autonomous system.
At the moment there are two integration methods:

* [Use the ROS bridge](getting-started-with-ros.md) to connect your ROS autonomous system to the bridge.
* [Use the Python client](getting-started-with-python.md) to interact with the simulator.
