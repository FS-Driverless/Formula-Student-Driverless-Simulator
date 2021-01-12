import sys
import os

## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import fsds
import numpy as np

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection, exit if not connected
client.confirmConnection()

# After enabling api controll only the api can controll the car. 
# Direct keyboard and joystick into the simulator are disabled.
client.enableApiControl(True)

# Data stucture for controlling the car
car_controls = fsds.CarControls()

# -1 is full steering left, +1 is full right, 0 is neutral
car_controls.steering = -0.7

# 0 is no throttle, 1 is full throttle
car_controls.throttle = 0

# 0 brake is no break, 1 is full break
car_controls.brake = 0

while (True):
    # Get the car state
    car_state = client.getCarState()
    
    print('car speed (m/s): {0}'.format(car_state.speed))
    
    if (car_state.speed < 5):
        car_controls.throttle = 0.5
    else:
        car_controls.throttle = 0.0

    print('throttle = {0}'.format(car_controls.throttle))
    
    client.setCarControls(car_controls)
    time.sleep(0.5)