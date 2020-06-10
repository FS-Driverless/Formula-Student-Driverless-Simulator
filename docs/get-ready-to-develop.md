# Let's get ready to develop this project
So you want to make changes to this project, amazing! We always love new developers 💓
Using this tutorial you can set up your computer to development the simulator. 
*If you just want to run the simulator, you can follow [this guide on how to simulate](how-to-simulate.md).*

## Prerequisites

For developing this project, you need quite a good computer because Unreal Engine is a heavy baby.
We highly recommend the following computer specs. You might be able to run with less power but everything will be slower.

* 8 core 3Ghz CPU
* 12 GB memory
* 100GB free SSD storage
* Recent NVidia card with Vulkan support and 3 GB of memory.

If your computer does not suffice you can use a remote workstation on Google Cloud Platform.
Read [this tutorial](gcp-remote-workstation.md) on how to setup your virtual workstation.

To check the video card drivers, run `vulkaninfo`. It should output a bunch of lines without errors.

[You should also enable Windows Subsystem for Linux 1 (WSL1)](https://docs.microsoft.com/en-us/windows/wsl/install-win10). 
No need to update to version 2.
Install ubuntu 18.04 LTS.
If you are on windows server, enable windows susbsystem for linux in the server manager and [install ubuntu](https://docs.microsoft.com/en-us/windows/wsl/install-on-server#download-a-linux-distribution).

## Install Unreal Engine (Windows)
Go to [unrealengine.com](https://www.unrealengine.com/) and download the epic installer.
You need an account for this.
Install the epic installer.

Launch the epic installer and install Unreal Engine 4.25

## Install visual studio 2019 (Windows)
[Download visual studio 2019 (community edition)](https://visualstudio.microsoft.com/vs/)

During installation, choose the following components:

* Desktop development with C++
* Game development with C++
* Linux development with C++

At 'Invidual Components select:

* C++ CMake tools for Windows
* Windows 10 SDK 10.0.18.362.0
* .NET Framework 4.7 SDK


## Install ROS Melodic (WSL Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
```

Add the following line to end of your `~/.bashrc` file:
```
source /opt/ros/melodic/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros/devel/setup.bash
```

### Gui applications from WSL Ubuntu in Windows
By default, if you are running Windows Subsystem for Linux with Ubuntu, you can't run gui applications.
This is super annoying if you want to use rqt applicatoins like rviz or rqt_plot.
It is easy to get this working though!
Just install [Xming](https://sourceforge.net/projects/xming/) on windows, and run it.
Next, go into the Ubuntu terminal and run `export DISPLAY=:0`.
Now you can run any all them gui apps!
You can even add `export DISPLAY=:0` to your `~/.bashrc` to always be able to use gui apps without having to manually run export.

## Clone the project

In Windows, install [git](https://git-scm.com/download/win) and [git lfs](https://git-lfs.github.com/).

Once both are installed, open git bash and run `git lfs install`.

Now clone the repo in the windows home directory 
```
git clone git@github.com:FS-Online/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

Go into the cloned repo and run `git config core.fileMode false` to ignore file mode changes.

**THE REPO HAS TO BE CLONED IN THE HOME DIRECTORY!**. So the repo location should be `$HOME/Formula-Student-Driverless-Simulator`.
Why you ask? Because we couldn't get relative paths in the C++ code to work so now we have hard-coded some paths to the home directory.
I know yes it is ugly but it works. If you are bothered by it I would welcome you to open a pr with a fix.

In Ubuntu wsl, create a symlink from `~/Formula-Student-Driverless-Simulator` to `~/Formula-Student-Driverless-Simulator`
```
ln -s /mnt/c/Users/developer/Formula-Student-Driverless-Simulator ~/Formula-Student-Driverless-Simulator
```

## What's next?
[Read here how to develop!](how-to-develop.md)