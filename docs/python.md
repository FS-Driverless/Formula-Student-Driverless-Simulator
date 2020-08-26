# Python API for FSDS

This package contains Python APIs for Formula Student Driverless Simulator.
It connects to the rpc api to retrieve sensordata and send vehicle controll setpoints.

## Dependencies
This package depends on msgpack, numpy and opencv-contrib. Install the dependencies like this:

```
pip install -r requirements.txt
```

## Getting started

Let's drive the car forward!

```
# This code adds the fsds package to the pyhthon path.
# It assumes the fsds repo is cloned in the home directory.
# Replace fsds_lib_path with a path to wherever the python directory is located.
import sys, os
fsds_lib_path = os.path.join(os.path.expanduser("~"), "Formula-Student-Driverless-Simulator", "python")
sys.path.insert(0, fsds_lib_path)

import time
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

# After enabling api controll only the api can controll the car. 
# Direct keyboard and joystick into the simulator are disabled.
# If you want to still be able to drive with the keyboard while also 
# controll the car using the api, call client.enableApiControl(False)
client.enableApiControl(True)

# Instruct the car to go full-speed forward
car_controls = fsds.CarControls()
car_controls.throttle = 1
client.setCarControls(car_controls)
```

[Find more examples here.](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples)