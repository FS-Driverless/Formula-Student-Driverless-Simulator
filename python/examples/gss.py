import sys
import os

"""
This example shows how to retrieve ground speed sensor data.
Before running this you must add the sensor to your vehicle.
Add the following to your settings.json file in the Sensors section:

    "GSS" : {
        "SensorType": 7,
        "Enabled": true
    },

"""


## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

while True:
    gss = client.getGroundSpeedSensorData(vehicle_name='FSCar')

    # nanosecond timestamp of when the gss data was captured
    print("timestamp nano: ", gss.time_stamp)

    # velocity in m/s of the car in world reference frame
    print("linear_velocity: ", gss.linear_velocity)

    print()

    time.sleep(1)
