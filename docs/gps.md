# GPS

The GPS model in AirSim has been configured to mimic an average GPS receiver used during formula student competitions.

The GPS operates at 10 Hz, this value can be altered with update_frequency parameter. 

At this moment there is no artificial latency added.
Still there will be some delay between the creation of the gps points and arrival at the autonomous system because of network latency.  
In the future, this can be altered with update_frequency parameter.         

All GPS positions are timestamped.

As soon as the car starts, gps becomes available at maximum resolution. 
There is no warmup time since during physical formula student events cars often start with pre-locked gps.
 
The inaccuracies in the GPS sensor position is generated with adding an offset vector from the ground truth, as a vector [x_err, y_err, z_err]T , where x_err, y_err are generated through a Gaussian distribution with 0 mean and eph variance, and z_err through a Gaussian distribution with 0 mean and epv variance.
Currently the eph is set to **4 cm**.  
This noise, however, is only added if the measured velocity is above an arbitrarily chosen threshold (currently 0.1m/s). 
To velocities below that, this additional inaccuracy is not introduced to avoid “jumpy” positions during standstill of the vehicle. 

See GpsSimple.hpp (/AirSim/AirLib/include/sensors/gps/GpsSimple.hpp) and [GpsSimpleParams.hpp](/AirSim/AirLib/include/sensors/gps/GpsSimpleParams.hpp) for the implementation of the gps model.

