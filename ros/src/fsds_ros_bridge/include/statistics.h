#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>

namespace ros_bridge
{
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> HighResClock;
    typedef std::chrono::duration<float> Duration;
    class Statistics
    {
        /* See statistics.md in the docs/ folder for more information about 
    this class and how it is used */

    public:
        // Having the default constructor is necessary for the workspace to build!
        Statistics(){};
        Statistics(const std::string name) : _statisticsType(name){};

        void Print()
        {
            // print statistics summary to the console
            std::cout << "-----------------------------------------------------\n";
            std::cout << "Printing statistics for: " << _statisticsType << "\n";
            if (_rosMsgCount != 0)
            {
                std::cout << "ROS msgs/s: " << _rosMsgCount << "\n";
            }
            if (!_durationHistory.empty())
            {
                float max_latency = *std::max_element(_durationHistory.begin(), _durationHistory.end());
                std::cout << "Max latency Rpc call: " << max_latency << "\n";
            }
            std::cout << "-----------------------------------------------------\n";
        }

        void addDurationRecording(const Duration &duration)
        {
            _durationHistory.emplace_back(duration.count());
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
        std::string _statisticsType;
        uint _rosMsgCount;
        std::vector<float> _durationHistory{};
    };

    class Timer
    {
    public:
        Timer(Statistics &statistics) : _statistics(statistics)
        {
            _start = std::chrono::high_resolution_clock::now();
        };

        ~Timer()
        {
            _end = std::chrono::high_resolution_clock::now();
            // Append the duration to the appropriate vector
            _statistics.addDurationRecording(std::move(_end - _start));
        }

    private:
        HighResClock _start, _end;

        Statistics _statistics;
    };

    class ROSMsgCounter
    {
    public:
        ROSMsgCounter(Statistics &statistics) : _statistics(statistics){};

        ~ROSMsgCounter()
        {
            std::cout << "Incrementing count! \n";
            // Increment message count
            _statistics.addCount();
        }

    private:
        Statistics _statistics;
    };

} // namespace ros_bridge