This is a Formula Student Driverless Simulation Competition System.

# Installation
0. Install gcc-8
```
sudo apt-get install gcc-8 g++-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8
```

1. Install Unreal Engine 4.24. You can install it anywhere on your computer.
```
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```
2. Install ROS Melodic

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop ros-melodic-mavros ros-melodic-tf2-geometry-msgs python-catkin-tools
```

3. Clone this repo
4. Run `AirSim/setup.sh` and  `AirSim/build.sh`
5. Run UE4Editor and open the `UE4Project/FSOnline.uproject` project.
6. Press 'play' and it should work!
7. open a new terminal, go into `Simulator` and run `catkin init`, `catkin build`
8. Start playing with ros!

# Repo Overview

`/UE4Project` is the Unreal Engine 4 project.
In here you will find everything that runs inside unreal engine is located.
This includes the world, all assets and the AirSim server.
The AirSim server uses the AirLib shared code (see `/AirSim/AirLib`).

`/AirSim` is a slimmed down, hard-fork of the [AirSim](https://github.com/microsoft/AirSim) project.
There is only code located that is shared between Simulator and UE4 plugin.
When AirSim is compiled, the AirLib binaries are placed within `/UE4Project/Plugins/AirSim/Source/AirLib`.

`/Simulator` is the simulation controll system. This is a ros workspace.

## Development workflow

If you make changes to the AirSim/AirLib code, you have to run `AirSim/build.sh`.
This will compile the AirLib into binaries and copy those binaries to the FSOnline UE4 project plugin.

If you make changes to the AirSim plugin (located in `FSOnline/Plugins/AirSim`) then you have to recompile the plugin.
This is also required if the AirLib binaries have changed.
To recompile, go into the UE4Editor, go to `Window` -> `Developer tools` -> `Modules`. Search for `AirSim` and click `Recompile`.
This will recompile and reload the plugin.

