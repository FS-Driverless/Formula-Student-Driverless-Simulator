## IMU Sensor
The IMU captures the acceleration, orientation and angular rate of the vehicle.
The IMU sensor in FSDS is using AirSim's built in IMU sensor simulation, that has been modelled and parametrized according to MPU 6000 IMU from [InvenSense](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).




The maximum achievable internal IMU frequency is 1000 Hz, ouputing information of the vehicle's 3-axis angular velocity, 3-axis linear acceleration, as well as its orientation in quaternions. 

IMU gyroscope and accelomter bias and accuracy parameters can be found and fine-tuned in AirLib/include/sensors/imu/ImuSimpleParams.hpp, or can be initialized with custom parameters in your `settings.json` file. 

The angular velocity, linear acceleration outputs as well as their biases have artificially introduced Gaussian noise (0 mean, standard deviation of 1) updated on each IMU cycle. 

All of the IMU measurements are timestamped.

The GPS is located in the [vehicle pawn center](vehicle_model.md). 

See ImuSimple.hpp (/AirSim/AirLib/include/sensors/imu/ImuSimple.hpp) and [ImuSimpleParams.hpp] /AirSim/AirLib/include/sensors/imu/ImuSimpleParams.hpp for the implementation of the IMU model.



