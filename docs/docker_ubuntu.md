# FSDS Simulator on Docker in Linux
This guide describes how to build a Docker image from [FSDS linux binaries](#binaries). 
You could instead compile Unreal Engine + AirSim from source, but this is not documented.
This Docker implementation is based on Micorsoft's Docker implementation for AIRSIM repository.

## Binaries
#### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))
- Install [Ros-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Get FSDS Repo same version as binary used [Ros-Bridge](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)

#### Build the Docker image
Use the `build_airsim_image.py` to build the docker image

NVIDIA vulkan is thebase image on which FSDS is installed.
By default Ubuntu 18.04 with CUDA 10.0 is used.
You can specify any [NVIDIA vulkan](https://hub.docker.com/r/nvidia/cudagl/) at your own risk.
Use `--target_image` is the desired name of your Docker image. It defaults to `fsdsairsim_binary` with tag as `vulkan-ubuntu18.04`
To customize the version, use:

```bash
$ cd docker/src;
$ python build_airsim_image.py --target_image=fsdsairsim_binary:vulkan-ubuntu18.04
```

After building the image, verify that the image exists by runnig:

```bash
    $ docker images | grep fsdsairsim_binary
```

#### Run FSDS inside the Docker container 
Get [the binary](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip), or package your own project in Ubuntu. 
You can either download the latest version yourself, or use the `download_FSDSSimulator_binary.sh` helper script.

Now, run fsds inside the Docker container:

```bash
   $ ./run_airsim_image_binary.sh DOCKER_IMAGE_NAME UNREAL_BINARY_SHELL_SCRIPT UNREAL_BINARY_ARGUMENTS -- headless
```

Replace the variables as follows:
   * `DOCKER_IMAGE_NAME`: Same as `target_image` parameter in previous step. By default, enter `fsdsairsim_binary:vulkan-ubuntu18.04`   
   * `UNREAL_BINARY_SHELL_SCRIPT`: for FSDSsimulator enviroment, it will be `fsds-v2.2.0-linux/FSDS.sh`
   * `UNREAL_BINARY_ARGUMENTS`: For FSDSsimulator, most relevant would be `-windowed`, `-ResX`, `-ResY`. [See here all options](https://docs.unrealengine.com/en-us/Programming/Basics/CommandLineArguments).

For FSDSsimulator, you can do a `$ ./run_airsim_image_binary.sh fsdsairsim_binary:vulkan-ubuntu18.04 fsds-v2.2.0-linux/FSDS.sh -windowed -ResX=1080 -ResY=720`

To run in headless mode, use suffix `-- headless` at the end:

```bash
$ ./run_airsim_image_binary.sh fsds-v2.2.0-linux/FSDS.sh -- headless
```

You need to have a `settings.json` file in the current working directory from where you run `./run_airsim_image_binary.sh`.