# How to develop this project
*Just looking to run the simulation and not interested in development of the simulator? Get started faster with the [simulation guide](how-to-simulate.md).*

This guide describes how to work on this project.
Before you continue reading, make sure you have finished the [get-ready-to-develop](get-ready-to-develop.md) guide.

As you can read in the [system overview](system-overview.md) this system consists of multiple components.


To develop any of these components you have to run Unreal Engine with the AirSim plugin.
So we need to compile the AirSim plugin.

## Compile the AirSim plugin and launch Unreal Engine
The AirSim plugin sourcode is made up of AirLib (/AirSim/AirLib) and the plugin code (/UE4Project/Plugins/AirSim/Source).
AirLib is also used by the ROS bridge.

First build the AirLib code. Open the _Developer Command Prompt for VS 2019_, go to `Formula-Student-Driverless-Simulator/AirSim` and run
```
build.cmd
```
The first time this takes quite a while. Go walk around a bit, maybe start playing [factoryidle](https://factoryidle.com/). 

After it is finished, launch unreal engine and open the project file `Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject`

It might show an error like 'This project was made with a different version of the Unreal Engine'. In that case select `more options` and `skip conversion`.

When asked to rebuild the 'Blocks' and 'AirSim' modules, choose 'Yes'.
This is the step where the plugin part of AirSim is compiled.

After some time Unreal Engine will start and you can launch the game.

If you make changes to the plugin code you only have to recompile the plugin. 
This can be done from within the Unreal Editor. go to to `Window` -> `Developer tools` -> `Modules`. 
Search for `AirSim` and click `Recompile`.

If you make changes to AirLib you have to run `build.cmd` again.


## ROS development

Read [here](building-ros.md) how to compile the ros workspace.


## Export the Unreal Engine project for release
1. Open the UE4Project in the Unreal Editor
3. Ensure 'File' -> 'Package Project' -> 'Build configuration' is set to 'Shipping,
2. Choose 'File' -> 'Package Project' -> 'Windows (64 bit)'
3. Select any folder on your computer.
4. Wait until it finishes.
5. Go into the `WindowsNoEditor` folder and rename `Blocks.exe` to `FSDS.exe`
6. Zip all files and upload to github release!

## Deploying documentation
For documentation we use [mkdocs](https://www.mkdocs.org/) and [mike](https://github.com/jimporter/mike/).
Hosting is provided by github pages. 

To tag a new version of the documentation and release it to github, first checkout the version that you want to deploy.
Then run `mike deploy VERSION latest -u -p`.
This will compile the documentation, store it as a new version in the `gh-pages` branch, update teh `latest` alias to point at the new version and push the `gh-pages` branch to github and thus making the documentation public.
To create a new version without updating the `latest` tag, omit the `latest -u` part. 

