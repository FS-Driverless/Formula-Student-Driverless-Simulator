# IMU Sensor
The IMU captures the acceleration, orientation and angular rate of the vehicle.
The IMU sensor in FSDS is using AirSim's built in IMU sensor simulation, that has been modelled and parametrized according to MPU 6000 IMU from [InvenSense](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).


The maximum achievable internal IMU frequency is 1000 Hz, ouputing information of the vehicle's 3-axis angular velocity, 3-axis linear acceleration, as well as its orientation in quaternions. 
Keep in mind that at the moment the car position inside the simulation is updated at the simulator's framerate.
IMU noise is added every time the IMU frame is requested.
Requesting imu frequency at a higher rate then the simulators framerate will result in different data every time but all based on the same ground truth position.

IMU gyroscope and accelomter bias and accuracy parameters can be found and fine-tuned in AirLib/include/sensors/imu/ImuSimpleParams.hpp.
There is an [open issue](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/238) to make these configurable throught the `settings.json`.

The angular velocity, linear acceleration outputs as well as their biases have artificially introduced Gaussian noise (0 mean, standard deviation of 1) updated on each IMU cycle. 

All of the IMU measurements are timestamped.

The IMU is located in the [vehicle pawn center](vehicle_model.md). 

See ImuSimple.hpp (/AirSim/AirLib/include/sensors/imu/ImuSimple.hpp) and [ImuSimpleParams.hpp] /AirSim/AirLib/include/sensors/imu/ImuSimpleParams.hpp for the implementation of the IMU model.

## Add an IMU to the car

In your `settings.json`, add the following to the `Sensors` object within the vehicle configuration:
```json
"Imu" : {
    "SensorType": 2,
    "Enabled": true
},
```
The key of the object, in this case `Imu` is the name of the sensor: this imu sensor is named `Imu`.
Use this name to retrieve sensordata.

At the moment no configuration of noise is supported.


## Python client

```python
# Args:
#   imu_name (str, optional): Name of IMU to get data from, specified in settings.json. When no name is provided the last imu will be used.
#   vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to.
# 
# Returns:
#   time_stamp (np.uint64) nanosecond timestamp of when the imu data was captured
#   orientation (Quaternionr) rotation of the sensor, relative to the northpole. It's like a compass
#   angular_velocity (Vector3r) how fast the car is rotating along it's three axis in radians/second
#   linear_acceleration (Vector3r) how fast the car is accelerating in meters/s2^m
#   
imu_data = client.getImuData(imu_name = '', vehicle_name = 'FSCar')
```


An example of how to collect imu data:

```python
import time
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

while True:
    imu = client.getImuData(imu_name = 'Imu', vehicle_name = 'FSCar')

    print("timestamp nano: ", imu.time_stamp)
    print("orientation: ", imu.orientation)
    print("angular velocity: ", imu.angular_velocity)
    print("linear acceleration: ", imu.linear_acceleration)
    print()

    time.sleep(1)
```
Full example [here](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples/imu.py).
