import setup_path 
import airsim
import numpy as np
import os
import time

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()

while True:
    res=client.getGpsData(gps_name='Gps', vehicle_name='FSCar')
    print res
    time.sleep(1)
