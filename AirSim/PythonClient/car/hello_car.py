import setup_path 
import airsim
import cv2
import numpy as np
import os
import time

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

client.updateCamera("FSCar", "front_left_custom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

client.enableApiControl(False)


            
