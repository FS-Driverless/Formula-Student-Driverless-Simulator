# Ground Speed Sensor (GSS)

The ground speed sensor is modeled around the Kistler ground speed (like the Kistler Correvit SFII).

Velocity information is captured in the global world frame in ENU frame.

At this moment no extra noise is added to the sensordata since the kistler 250hz data averaged into the 100hz is so close to ground truth that adding noise would be unrealistic.

## Add the sensor to the car

In your `settings.json`, add the following to the `Sensors` object within the vehicle configuration:

```json
"GSS" : {
  "SensorType": 7,
  "Enabled": true
}
```
A car can have no more then 1 gss.
Therefore, the name of the gss (in this example `GSS`) doesn't matter; for requesting gss data you don't need to pass a sensorname.

## Python

```python
# Args:
#   vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to.
# Returns:
#   time_stamp (np.uint64): nanosecond timestamp of when the gss data was captured
#   linear_velocity (Vector3r): velocity in m/s of the car in world reference frame
gss_data = client.getGroundSpeedSensorData(vehicle_name = 'FSCar')
```

An example of how to collect gss data:

```python
import time
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

while True:
    gss = client.getGroundSpeedSensorData(vehicle_name='FSCar')

    print("timestamp nano: ", gss.time_stamp)
    print("linear_velocity: ", gss.linear_velocity)

    print()

    time.sleep(1)
```

Full example [here](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples/gss.py).


## Ros
When using the ros bridge, ground speed sensordata will be published on `/fsds/gss` with the `geometry_msgs/TwistStamped` message type.
Make sure you have added the sensor to your settings.json file.

Appart from the header fields, only `x`, `y` and `z` of the `twist.linear` are populated. 

```yaml
header:
  seq: 5747
  stamp:
    secs: 1595325426
    nsecs: 617730500
  frame_id: "fsds/FSCar"
twist:
  linear:
    x: 4.80838251114
    y: -0.0
    z: -0.0214105024934
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
```