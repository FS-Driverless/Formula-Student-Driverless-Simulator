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
```
"Gps" : {
    "SensorType": 3,
    "Enabled": true
},
```
At the moment no configuration is allowed

## Python client

```
gps_data = client.getGpsData(gps_name = "", vehicle_name = "")
```

An example to show how to collect gps data:

```
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
```