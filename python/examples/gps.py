import sys
import os

"""
This example shows how to retrieve gps sensordata.
Before running this you must add the sensor to your vehicle.
Add the following to your settings.json file in the Sensors section:

    "Gps" : {
        "SensorType": 3,
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
    gps = client.getGpsData(gps_name='Gps', vehicle_name='FSCar')

    # nanosecond timestamp of when the gps position was captured
    print("timestamp nano: ", gps.time_stamp)

    # UTC millisecond timestamp of when the gps position was captured
    print("timestamp utc:  ", gps.gnss.time_utc)

    # Standard deviation of horizontal position error (meters)
    print("eph: ", gps.gnss.eph)

    # Standard deviation of vertical position error (meters)
    print("epv: ", gps.gnss.epv)

    # The altitude, latitude and longitude of the gps
    print("geo point: ", gps.gnss.geo_point)

    # velocity in three directions (x_val, y_val and z_val) in meter/second
    print("velocity: ", gps.gnss.velocity)

    print()

    time.sleep(1)
