# Getting started

When running this simulator there are two main components: the simulator and the ros bridge.
Your autonomous system and the ros bridge should use the same ros core, either by running on the same computer or through a network connection. 
It is required that the simulator and the ros bridge have the same version!

This page is an overview of the different methods to get these components up and running.

## Running the simulator

To run the simulation smoothly you need quite a fast windows computer with a modern videocard.
We highly recommend the following computer specs. You might be able to run with less power but everything will be slower.
For developing this project, you need quite a good computer because Unreal Engine is a heavy baby.

* 8 core 2.3Ghz CPU
* 12 GB memory
* 30GB free SSD storage (120GB when building the unreal project from source)
* Recent NVidia card with Vulkan support and 3 GB of memory. (You can check the video card drivers by running `vulkaninfo`)

If your computer does not suffice you can use a remote workstation on Google Cloud Platform.
Read [this tutorial](gcp-remote-workstation.md) on how to setup your virtual workstation.

The simulator will load settings from the file `Formula-Student-Driverless-Simulator/settings.json` in your **home directory**.
This file is required for the simulator to work and contains the sensor configuration of the car.
If you clone the repo you will already have this file in place.
If not, copy-paste the contents of the [settings.json file at the root of this repository](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/blob/master/settings.json) into the `~/Formula-Student-Driverless-Simulator`.
This should get you started with the default sensor configuration, feel free to try your own custom sensor suite.

### From release binaries

The simulator is distributed as binaries on every release.
[At this moment only windows binaries are released. For now, if you are on Ubuntu you will have to run the simulator from the Unreal Engine Editor.](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/issues/107)
During competition, the simulation will run on Windows because it offers a bit better performance and stability.

Go to [releases](https://github.com/FS-Online/Formula-Student-Driverless-Simulator/releases) and download the latest one.
Unzip it to anywhere on your computer and launch FSDS.exe.
A window with a car should popup!
Try driving the car around using the arrowkeys.
If you get a black screen with some buttons, make sure the folder with the binary is in your user folder (Windows: `C:\Users\username\Formula-Student-Driverless-Simulator`, Linux: `~/Formula-Student-Driverless-Simulator`)

### From source using the Unreal Engine Editor

If you want to run the unreal engine project from source you will need [unreal engine and visual studio 2019](software-install-instructions.md).
On Ubuntu you can skip the visual studio 2019 part.

#### Get the repository

You can either download the repo using the big green download button on the [github page of this project](https://github.com/FS-Online/Formula-Student-Driverless-Simulator) or clone the repository. For cloning, checkout the documentation on this further down this page.

When downloading or cloning, by default you get the latest, unreleased version. This is probably not the version that you want. Make sure you select the version that you need! 

#### Compiling the AirSim plugin
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

#### Working with the Unreal Engine project

Launch Unreal Engine and open the project file `Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject`

It might show an error like 'This project was made with a different version of the Unreal Engine'. In that case select `more options` and `skip conversion`.

When asked to rebuild the 'Blocks' and 'AirSim' modules, choose 'Yes'.
This is the step where the plugin part of AirSim is compiled.

After some time Unreal Engine will start and you can launch the game. 
Run the game in standalone mode or or selected viewport mode, simulate and eject mode do not support camera's.

If you make changes to AirLib you have to run `build.cmd` again.

If you make changes to the plugin code or AirLib, you only have to recompile the plugin.
This can be done from within the Unreal Editor. go to to `Window` -> `Developer tools` -> `Modules`.
Search for `AirSim` and click `Recompile`.

#### Launching the game

To run the game, click the big Play button.
If you want to run it like it would run when packaged, choose 'Run as standalone game'.

## Running the Ros Bridge

The simulator exposes an RPC api that is used by the ros bridge to communicate with the vehicle.
The ros bridge should preferable run on the same computer as the simulator to ensure low latency and high bandwidth.
**Note that if you are using 2 local machines, there are some limitations** [check this](local-setup.md)

It is possible to run the ros bridge on a different computer than the simulator.
To get this to work you should use the `host` argument in the `fsds_ros_bridge.launch` file.
The ros bridge will connect to the simulator on port 41451.

The ros bridge only works on Ubuntu.
If you have the simulator running on windows we reccommend Windows Subsystem for Linux.
This offers a virtual Ubuntu machine within Windows.
You can read [here how to install it](software-install-instructions.md). 
While you are at it you might also want to [install Xming](software-install-instructions.md) so you can run rviz and rqt_plot from within WSL.

### Requirements

The ros bridge requires [ROS Melodic to be installed](software-install-instructions.md), as well as the following dependencies:
```
sudo apt-get install ros-melodic-tf2-geometry-msgs python-catkin-tools ros-melodic-rqt-multiplot ros-melodic-joy ros-melodic-cv-bridge ros-melodic-image-transport libyaml-cpp-dev libcurl4-openssl-dev
```

Make sure you have git lfs installed!

### Cloning the repository

Ready? Lets clone the repo into your home directory:
```
git clone git@github.com:FS-Online/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

If you havn't setup your ssh keys, you can clone using https by running the following command:
```
git clone https://github.com/FS-Online/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

**THE REPO HAS TO BE CLONED IN THE HOME DIRECTORY!**. So the repo location should be `$HOME/Formula-Student-Driverless-Simulator`.
Why you ask? Because we couldn't get relative paths in the C++ code to work so now we have hard-coded some paths to the home directory.
I know yes it is ugly but it works. If you are bothered by it I would welcome you to open a pr with a fix.

If this folder already exists as a result of any previous step, move the existing folder out of the way and merge the content afterwards.

If you are on Windows and cloned this repository in a Windows directory, go into the cloned repo and run `git config core.fileMode false` to ignore file mode changes. 
If you want to share the the clone directory with the Ubuntu WSL system, create a symlink within WSL like so:
```
ln -s /mnt/c/Users/developer/Formula-Student-Driverless-Simulator ~/Formula-Student-Driverless-Simulator
```

Now, checkout the version equal to the simulator. If you are running for example simulator packaged version beta-3, run `git checkout beta-3` to get the ros brige to the same version

### Preparing AirLib

AirLib is the shared code between the ros wrapper and the AirSim Unreal Engine plugin.
We need to stage the source before we can compile it together with the wrapper.

If you are working in a WSL shared folder where previously build.cmd was ran, you can skip this step.

Open an Ubuntu terminal and run `AirSim/setup.sh`.
This will download the nessesary libraries required to compile AirLib.
You will only need to run this once.

Everything setup.sh does is also included in build.cmd.

### Building the workspace

```
cd ros
catkin init
catkin build
```

### Launching the ros bridge

The ros bridge consists of a few nodes to achieve the highest performance and keep the codebase clean.
Everything can be launched using the `fsds_ros_bridge.launch` launchfile.
```
cd ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```

The ros bridge will read the settings from `~/Formula-Student-Driverless-Simulator/settings.json`.
Make sure this is the same configuration file as the simulator uses.

[Read all about configuring the ros bridge here.](ros-bridge.md)