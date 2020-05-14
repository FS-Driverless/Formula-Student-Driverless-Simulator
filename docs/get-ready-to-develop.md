# Let's get ready to develop this project
So you want to make changes to this project, amazing! We always love new developers <3
Using this tutorial you can set up your local development environment.

## Prerequisites
To run this project you need quite a good computer with a modern Nvidia video card.
To check the video card drivers, run `vulkaninfo`. It should output a bunch of lines without errors.

Also, make sure you have 100GB free disk space. 
You also need to run **ubuntu 18.04 LTS**. Any other platforms are not supported.
If your computer does not suffice you can use a remote workstation on Google Cloud Platform.
Read [this tutorial](gcp-remote-workstation.md) on how to setup your virtual workstation.

We will be compiling all kinds of C++ files and it is important that they are all compiled using the same compiler version. 
Therefore we will install GCC 8 and set it to use it when `gcc` is called.

```
sudo apt-get install gcc-8 g++-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8
```

## Install Unreal Engine
We use Unreal Engine 4.24. You can install it anywhere on your computer.
```
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh && ./GenerateProjectFiles.sh && make
```

Try to run it by executing `./Engine/Binaries/Linux/UE4Editor`. It should show the editor without errors.

## Install ROS Melodic

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop ros-melodic-mavros ros-melodic-tf2-geometry-msgs python-catkin-tools
```

## Install the project

This repo uses git LFS. So ensure you have LFS installed: `sudo apt-get install git-lfs`. 
Once git-lfs is downloaded, `git lfs install` must be run.

Let's start by cloning this repository in the home directory
```
git clone git@github.com:FS-Online/Driverless-Competition-Simulator.git
```

**THE REPO HAS TO BE CLONED IN THE HOME DIRECTORY!**. So the repo location should be `~/Driverless-Competition-Simulator`.
Why you ask? Because we couldn't get relative paths in the C++ code to work so now we have hard-coded some paths to the home directory.
I know yes it is ugly but it works. If you are bothered by it I would welcome you to open a pr with a fix.


Now we have to build the AirSim libraries:
```
cd ~/Driverless-Competition-Simulator
AirSim/setup.sh
AirSim/build.sh
```

To set up the ROS workspace, cd into the `ros` folder and run
```
catkin init
catkin build
```

Finally, you can launch the FSOnline Unreal Engine Project by running UE4Editor and selecting the `UE4Project/FSOnline.uproject` project.
To do this in a single command you can run `UE4Editor Driverless-Competition-Simulator/UE4Project/FSOnline.uproject`


## Development workflow

If you make changes to the AirSim/AirLib code, you have to run `AirSim/build.sh`.
This will compile the AirLib into binaries and copy those binaries to the FSOnline UE4 project plugin.

If you make changes to the AirSim plugin (located in `FSOnline/Plugins/AirSim`) then you have to recompile the plugin.
This is also required if the AirLib binaries have changed.
To recompile, go into the UE4Editor, go to `Window` -> `Developer tools` -> `Modules`. Search for `AirSim` and click `Recompile`.
This will recompile and reload the plugin.
