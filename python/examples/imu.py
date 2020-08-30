import sys
import os

"""
This example shows how to retrieve imu sensordata.
Before running this you must add the sensor to your vehicle.
Add the following to your settings.json file in the Sensors section:

    "Imu" : {
        "SensorType": 2,
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
    imu = client.getImuData(imu_name = 'Imu', vehicle_name = 'FSCar')

    # nanosecond timestamp of when the imu frame was captured
    print("timestamp nano: ", imu.time_stamp)

    #  rotation of the sensor, relative to the northpole. It's like a compass
    print("orientation: ", imu.orientation)

    # how fast the car is rotating along it's three axis in radians/second
    print("angular velocity: ", imu.angular_velocity)

    # how fast the car is accelerating in meters/s2^m
    print("linear acceleration: ", imu.linear_acceleration)

    print()

    time.sleep(1)
