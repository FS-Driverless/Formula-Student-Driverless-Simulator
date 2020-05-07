This is a Formula Student Driverless Simulation Competition System.
It will provide a virtual environment where Autonomous Systems from different Formula Student teams can compete in time-trial challenges. 
The competition will take place during the driverless event of the FS-Online 2020.

## Repo Overview

`/UE4Project` is the Unreal Engine 4 project.
In here you will find everything that runs inside unreal engine is located.
This includes the world, all assets and the AirSim server.
The AirSim server uses the AirLib shared code (see `/AirSim/AirLib`).

`/AirSim` is a slimmed down, hard-fork of the [AirSim](https://github.com/microsoft/AirSim) project.
There is only code located that is shared between Simulator and UE4 plugin.
When AirSim is compiled, the AirLib binaries are placed within `/UE4Project/Plugins/AirSim/Source/AirLib`.

`/Simulator` is the simulation controll system. This is a ros workspace.

This repo uses LFS for some large files. All files bigger then 90MB are added to LFS.

## Development

For developing within this repo you need quite a good computer because Unreal Engine is a heavy baby.
We recommand the following computer specs. You might be able to run with less power but everything will be slower.
* 8 core 3Ghz cpu
* 12 GB memory
* 100GB free SSD storage (required)
* Recent NVidia card with vulkan support and like 3 gb of memory.

If you do not have access to such a computer (like the most of) you can [setup a remote workstation in google cloud](docs/gcp-remote-workstation.md).

To actually setup your computer for development follow [this tutorial](docs/get-ready-to-develop.md).


## Credits
This project is based on the work of some amazing other open source projects. 

Primarily, [the AirSim project built by Microsoft and many open source contributors](https://github.com/microsoft/AirSim). 
Without it we could have never done this.

Many game assets like the surrounding world are based on assets from the [Formula Student Technion Driverless AirSim fork](https://github.com/FSTDriverless/AirSim). These assets made this project look as cool as it does now.


## License

Copyright (C) 2020 Driverless-Competition-Simulator Contributors

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
