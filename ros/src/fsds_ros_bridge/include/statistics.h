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
        - 
    - ROS callback frequency of the following subscriber:
        - 

    There will be one instance of this class for each Rpc call to be monitored. 
    To make a latency measurement, the appropriate instance pointer has to be passed
    to a new Timer class (see below):

    ros_bridge::Statistics rpcCallExample;

    { // Enter scope to be timed
        ros_bridge::Timer timer(&rpcCallExample); // Start timing

        // do the Rpc Call

    } // Go out of scope -> call the timer destructor 
     // which automatically stores the time elapsed in the instance of 
     // the class that was passed

    // TODO: make sure this wrapper is a template
    In the 1Hz ROS timer, the Print function will be called (or some wrapper which applies this action to all the instances) 
    followed by the Reset function(s) (or some wrapper which applies this action to all the instances). 


     */

    public:
        Statistics(const std::string &name) : _statisticsType(name){};

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

        void ResetCount()
        {
            _rosMsgCount = 0;
        }

        void ResetDurationHistory()
        {
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