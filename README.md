This is a Formula Student Driverless Simulation Competition System.

# Repo Overview

`/UE4Project` is the Unreal Engine 4 project.
In here you will find everything that runs inside unreal engine is located.
This includes the world, all assets and the AirSim server.
The AirSim server uses the AirLib shared code (see `/AirSim/AirLib`).

`/AirSim` is a slimmed down, hard-fork of the [AirSim](https://github.com/microsoft/AirSim) project.
Here is only code located that is shared between Simulator and UE4 plugin.
When AirSim is compiled, the AirLib binaries are placed within `/UE4Project/Plugins/AirSim/Source/AirLib`.


## Development workflow

If you make changes to the AirSim/AirLib code, you have to run `AirSim/build.sh`.
This will compile the AirLib into binaries and copy those binaries to the FSOnline UE4 project plugin.

If you make changes to the AirSim plugin (located in `FSOnline/Plugins/AirSim`) then you have to recompile the plugin.
This is also required if the AirLib binaries have changed.
To recompile, go into the UE4Editor, go to `Window` -> `Developer tools` -> `Modules`. Search for `AirSim` and click `Recompile`.
This will recompile and reload the plugin.

