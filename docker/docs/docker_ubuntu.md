# FSDS Simulator on Docker in Linux
We've two options for docker. You can either build an image for running [FSDS linux binaries](#binaries), or for compiling Unreal Engine + AirSim [from source](#source).
We will use only the Binary method due to the high load of source compilation in building Unreal engine sources.
This docker implementation is based on Micorsoft's Docker implementation for AIRSIM repository.

## Binaries
#### Requirements:
- Install [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))
- Install [Ros-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Get FSDS Repo same version as binary used [Ros-Bridge](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)

#### Build the docker image
- Below are the default arguments.   
   NVIDIA vulkan is the image over which we'll install FSDS simulator. We've tested on Ubuntu 18.04 with CUDA 10.0.  
   You can specify any [NVIDIA vulkan ](https://hub.docker.com/r/nvidia/cudagl/) at your own risk.    
   `--target_image` is the desired name of your docker image.    
   Defaults to `fsdsairsim_binary` with tag as `vulkan-ubuntu18.04`

```bash
$ cd docker/src;
$ python build_airsim_image.py \
   --target_image=fsdsairsim_binary:vulkan-ubuntu18.04
```

- Verify you have an image by:
 `$ docker images | grep fsdsairsim_binary`   

#### Running an unreal binary inside a docker container 
- Get [an unreal binary](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-linux.zip) or package your own project in Ubuntu.   
Let's take the fsdsAirsim v2.0 binary as an example.   
Update the download_FSDSSimulator_binary.sh in case of new version of binaries.
You can download it by running 

```bash
   $ cd docker/src;
   $ ./download_FSDSSimulator_binary.sh
```

- Running an unreal binary inside a docker container 
   The syntax is:

```bash
   $ ./run_airsim_image_binary.sh DOCKER_IMAGE_NAME UNREAL_BINARY_SHELL_SCRIPT UNREAL_BINARY_ARGUMENTS -- headless     
```

   For FSDSsimulator, you can do a `$ ./run_airsim_image_binary.sh fsdsairsim_binary:vulkan-ubuntu18.04 fsds-v2.0.0-linux/FSDS.sh -windowed -ResX=1080 -ResY=720`

   * `DOCKER_IMAGE_NAME`: Same as `target_image` parameter in previous step. By default, enter `fsdsairsim_binary:vulkan-ubuntu18.04`   
   * `UNREAL_BINARY_SHELL_SCRIPT`: for FSDSsimulator enviroment, it will be `fsds-v2.0.0-linux/FSDS.sh`
   * [`UNREAL_BINARY_ARGUMENTS`](https://docs.unrealengine.com/en-us/Programming/Basics/CommandLineArguments):
      For FSDSsimulator, most relevant would be `-windowed`, `-ResX`, `-ResY`. Click on link to see all options. 
         
  * Running in Headless mode:    
      Suffix `-- headless` at the end:
```bash
$ ./run_airsim_image_binary.sh fsds-v2.0.0-linux/FSDS.sh -- headless
```

- [Specifying a `settings.json`](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/settings.json)
