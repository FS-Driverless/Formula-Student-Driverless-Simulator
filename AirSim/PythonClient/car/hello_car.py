import setup_path 
import airsim
import numpy as np
import os
import time

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

client.jsonSettingsUpdate()

client.enableApiControl(False)


            
