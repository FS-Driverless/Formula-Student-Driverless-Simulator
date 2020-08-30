# GPS

The GPS model in AirSim has been configured to mimic an average GPS receiver used during formula student competitions.

The GPS operates at 10 Hz, this value can be altered with update_frequency parameter.

The GPS is located in the [vehicle pawn center](vehicle_model.md). 
The GPS captures the position of the vehicle in the geodetic reference system, namely longitude [deg], latitude [deg], altitude [m].

At this moment there is no artificial latency added.
Still there will be some delay between the creation of the gps points and arrival at the autonomous system because of network latency.  
In the future, this can be altered with update_frequency parameter.         

All GPS positions are timestamped.

As soon as the car starts, gps becomes available at maximum resolution. 
There is no warmup time since during physical formula student events cars often start with pre-locked gps.
 
The inaccuracies in the GPS sensor position is generated with adding an offset vector from the ground truth, as a vector [x_err, y_err, z_err]<sup>T</sup>, where x_err, y_err are generated through a Gaussian distribution with 0 mean and eph variance, and z_err through a Gaussian distribution with 0 mean and epv variance.
Currently the eph is set to **4 cm**.  
This noise, however, is only added if the measured velocity is above an arbitrarily chosen threshold (currently 0.1m/s). 
To velocities below that, this additional inaccuracy is not introduced to avoid "jumpy" positions during standstill of the vehicle. 

See GpsSimple.hpp (/AirSim/AirLib/include/sensors/gps/GpsSimple.hpp) and [GpsSimpleParams.hpp](/AirSim/AirLib/include/sensors/gps/GpsSimpleParams.hpp) for the implementation of the gps model.

## Add the gps to the car
In your `settings.json`, add the following to the `Sensors` object within the vehicle configuration:
```json
"Gps" : {
    "SensorType": 3,
    "Enabled": true
},
```
The key of the object, in this case `Gps` is the name of the sensor: this gps sensor is named `Gps`.
Use this name to retrieve sensordata.

At the moment no accuracy-configuration is supported.

## Python client

```python
# Args:
#   gps_name (str, optional): Name of GPS to get data from, specified in settings.json. When no name is provided the last gps will be used.
#   vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to.
# Returns:
#   time_stamp (np.uint64): nanosecond timestamp of when the gps position was captured
#   gnss:
#     eph (float): Standard deviation of horizontal position error (meters)
#     epv (float): Standard deviation of vertical position error (meters)
#     geo_point: The altitude, latitude and longitude of the gps
#       latitude (float)
#       longitude (float)
#       altitude (float)
#     velocity (Vector3r): Velocity in three directions (x_val, y_val and z_val) in meter/second
#     time_utc (np.uint64): UTC millisecond timestamp of when the gps position was captured
gps_data = client.getGpsData(gps_name = '', vehicle_name = 'FSCar')
```

An example of how to collect gps data:

```python
import time
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

while True:
    gps = client.getGpsData()

    print("timestamp nano: ", gps.time_stamp)
    print("timestamp utc:  ", gps.gnss.time_utc)
    print("eph: ", gps.gnss.eph)
    print("epv: ", gps.gnss.epv)
    print("geo point: ", gps.gnss.geo_point)
    print("velocity: ", gps.gnss.velocity)

    print()

    time.sleep(1)
```
Full example [here](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples/gps.py).
