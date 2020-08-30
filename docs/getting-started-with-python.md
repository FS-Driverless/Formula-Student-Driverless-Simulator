# Connecting to the simulator with Python

You can use the python client to connect the simulator.
The python client is able to retrieve sensordata and send vehicle controll.

## Dependencies
The Python client depends on msgpack, numpy and opencv-contrib. 
Install the dependencies like this:

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

time.sleep(5)

# Places the vehicle back at it's original position
client.reset()
```

A full example of an autonomous system that can finish a lap [can be found here](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples/autonomous_example.py)

[Find more examples here.](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples)

## Sensors

Documentation on requesting and processing sensordata can be found in the respective sensor documentation pages:

* [Lidar](lidar.md)
* [Camera](camera.md)
* [GPS](gps.md)
* [IMU](imu.md)
* [Ground Speed Sensor](ground-speed-sensor.md)

## Getting ground truth information

Using the following function you get get the real, latest position of the car:

```
state = client.getCarState()

# velocity in m/s in the car's reference frame
print(state.speed)

# nanosecond timestamp of the latest physics update
print(state.timestamp)

# position (meter) in global reference frame.
print(state.kinematics_estimated.position)

# orientation (Quaternionr) in global reference frame.
print(state.kinematics_estimated.orientation)

# m/s
print(state.kinematics_estimated.linear_velocity)

# rad/s
print(state.kinematics_estimated.angular_velocity)

# m/s^2
print(state.kinematics_estimated.linear_acceleration)

# rad/s^2
print(state.kinematics_estimated.angular_acceleration)
```