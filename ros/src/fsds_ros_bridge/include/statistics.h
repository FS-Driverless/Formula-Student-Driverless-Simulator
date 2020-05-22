#include <chrono>
#include <iostream>
#include <vector>

namespace ros_bridge
{
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> HighResClock;
    typedef std::chrono::duration<float> Duration;
    class Statistics
    {

        /* This class is used by the airsim ros wrapper class to monitor 
    performance of rpc calls as well as of the ROS network. The statistics
    gathered by the methods of this class and temporarily stored by private 
    class members will be printed live at about 1Hz. This will be useful 
    both for development of the simulator as well as for ensuring fairness
    in the competition, where a team will be allowed to retry a run if problems
    are diagnosed on the simulator side.
    
    We are gathering statistics about:
    - Rpc calls:
        - getGpsData
        - getCarState
        - getImuData
        - simGetImages
        - getLidarData
        - setCarControls
    - ROS publishing frequency of the following publishers:
        - odom_local_ned_pub
        - global_gps_pub
        - cam_info_pub_vec_
        - lidar_pub_vec_
        - imu_pub_vec_
    - ROS callback frequency of the following subscriber(s):
        - control_cmd_sub

    There will be one instance of this class for each Rpc 
    call to be monitored. To make a latency measurement, 
    the appropriate instance pointer has to be passed
    to a new Timer class (see below):

    ros_bridge::Statistics rpcCallStatistics;

    { // Enter scope to be timed
        ros_bridge::Timer timer(&rpcCallStatistics); // Start timing

        // do the Rpc Call

    } // Go out of scope -> call the timer destructor 
     // which automatically stores the time elapsed in the instance of 
     // the class that was passed

    There will also be an instance of this class for each ROS 
    publisher/subscriber. To count a new incoming or outgoing message the
    simple construct below can be used:

    ros_bridge::Statistics pubSubStatistics;


    For a publisher:

    {

    }

    For a subscriber:

    void callback(msg) {
        // pass pointer to persistent Statistics object
        ros_bridge::ROSMsgCounter counter(&pubSubStatistics); 

        // Do something with msg


    } // scope ends, destructor is called and count is incremented for 
      // the Statistics object

    In the 1Hz ROS timer, the Print function will be called 
    (or some wrapper which applies this action to all the instances) 
    followed by the Reset function(s) (or some wrapper which applies 
    this action to all the instances) which ensures that counters 
    are set to 0 and that vectors of durations (latencies) are emptied. 

     */

    public:
        Statistics(const std::string&& name) : _statisticsType(name){};

        void Print()
        {
            // print statistics summary to the console
            std::cout << "Printing statistics for: " << _statisticsType << "\n";
        }

        void addDurationRecording(const Duration &duration)
        {
            _durationHistory.emplace_back(duration);
        }

        void addCount()
        {
            ++_rosMsgCount;
        }

        // There is probably a better way of resetting the vector which prevents
        // allocating all the space needed for its elements again
        void Reset()
        {
            // Reset count
            _rosMsgCount = 0;

            // reset duration history
            _durationHistory = {};
        }

    private:
        const std::string _statisticsType;
        uint _rosMsgCount;
        std::vector<Duration> _durationHistory{};
    };

    class Timer
    {
    public:
        Timer(Statistics *statistics) : _statistics(statistics)
        {
            _start = std::chrono::high_resolution_clock::now();
        };

        ~Timer()
        {
            _end = std::chrono::high_resolution_clock::now();
            // Append the duration to the appropriate vector
            _statistics->addDurationRecording(std::move(_end - _start));
        }

    private:
        HighResClock _start, _end;

        Statistics *_statistics;
    };

    class ROSMsgCounter
    {
    public:
        ROSMsgCounter(Statistics *statistics) : _statistics(statistics){};

        ~ROSMsgCounter()
        {
            // Increment message count
            _statistics->addCount();
        }

    private:
        Statistics *_statistics;
    };

} // namespace ros_bridge