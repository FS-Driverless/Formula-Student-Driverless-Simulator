# ROS Bridge Monitoring

ROS Bridge and Rpc performance monitoring is done by the [Statistics](../ros/src/fsds_ros_bridge/include/statistics.h) class. This class is used by the `AirsimROSWrapper` class to monitor 
latency of rpc calls as well as the frequency of ceratain topics in the ROS network. The statistics
gathered by the methods of this class and temporarily stored by private 
class members will be printed live at about 1Hz. This will be useful 
both for development of the simulator as well as for ensuring fairness
in the competition, where a team will be allowed to retry a run if problems
are diagnosed on the simulator side. 

The following describes how the class and the auxiliary classes `Timer` and `ROSMsgCounter` measure performance and are implemented, so that as a developer you can monitor new publishers, subscribers, rpc calls, or actually any line of code in the AirSimROSWrapper class.

We are gathering statistics about:
- Rpc calls:
    - getGpsData
    - getCarState
    - getImuData (vector)
    - simGetImages (vector)
    - getLidarData (vector)
    - setCarControls
- ROS publishing frequency of the following publishers:
    - odom_local_ned_pub
    - global_gps_pub
    - cam_pub_vec_ (vector)
    - lidar_pub_vec_ (vector)
    - imu_pub_vec_ (vector)
- ROS callback frequency of the following subscriber(s):
    - control_cmd_sub

There will be one instance of this class for each Rpc 
call to be monitored. To make a latency measurement, 
the appropriate instance pointer has to be passed
to a new Timer class (see below):

```
ros_bridge::Statistics rpcCallStatistics = ros_bridge::Statistics("rpcCallName");

{ // Enter scope to be timed
    ros_bridge::Timer timer(&rpcCallStatistics); // Start timing

    // do the Rpc Call

} // Go out of scope -> call the timer destructor 
    // which automatically stores the time elapsed in the instance of 
    // the class that was passed
```
There will also be an instance of this class for each ROS 
publisher/subscriber. To count a new incoming or outgoing message the
simple construct below can be used:
```
ros_bridge::Statistics pubSubStatistics = ros_bridge::Statistics("pubSubName");;

// For a publisher:
{
    // pass persistent Statistics object (by reference)
    ros_bridge::ROSMsgCounter counter(&pubSubStatistics); 
    pubSub.publish(data);
}

// For a subscriber:

void callback(msg) {
    // pass persistent Statistics object (by reference)
    ros_bridge::ROSMsgCounter counter(&pubSubStatistics); 

    // Do something with msg


} // scope ends, destructor is called and count is incremented for 
    // the Statistics object
```
In the 1Hz ROS timer, the Print function will be called 
(the wrapper which applies this action to all the instances) 
followed by the Reset function (the wrapper which applies 
this action to all the instances) which ensures that counters 
are set to 0 and that vectors of durations (latencies) are emptied.