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
                double ros_msg_hz = _time_elapsed == 0.0f ? 1 : _rosMsgCount/_time_elapsed;
                std::cout << "ROS msgs/s: " << ros_msg_hz << "\n";
            }
            if (!_durationHistory.empty())
            {
                float max_latency = *std::max_element(_durationHistory.begin(), _durationHistory.end());
                std::cout << "Max latency Rpc call: " << max_latency << "us\n";
            }
            std::cout << "-----------------------------------------------------\n";
        }

        void addDurationRecording(const Duration &duration)
        {
            // std::cout << "[Statistics] adding recording: " << duration.count() << "\n";
            _durationHistory.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
        }

        void addCount()
        {
            ++_rosMsgCount;
            // std::cout << "Count incremented to: " << _rosMsgCount << "\n";
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

        uint getCount() const {
            return _rosMsgCount;
        }

        static void SetTimeElapsed(const double time_elapsed) {
            Statistics::_time_elapsed = time_elapsed;
        }

    private:
        inline static double _time_elapsed = 1.0f;
        std::string _statisticsType;
        uint _rosMsgCount;
        std::vector<float> _durationHistory{};
    };

    

    class Timer
    {
    public:
        Timer(Statistics* statistics) : _statistics(statistics)
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

        Statistics* _statistics;
    };

    class ROSMsgCounter
    {
    public:
        ROSMsgCounter(Statistics* statistics) : _statistics(statistics){};

        ~ROSMsgCounter()
        {
            // std::cout << "[RosMsgCounter] Incrementing count of object " << _statistics <<  "\n";

            // Increment message count
            _statistics->addCount();

            // std::cout << "[RosMsgCounter] Count: " << _statistics->getCount() << "\n";
        }

    private:
        Statistics* _statistics;
    };

} // namespace ros_bridge