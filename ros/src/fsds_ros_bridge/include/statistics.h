#include <chrono>
#include <iostream>
#include <vector>

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
            std::cout << "-----------------------------------------------------"
            std::cout << "Printing statistics for: " << _statisticsType << "\n";

            std::cout << "-----------------------------------------------------"
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
        std::string _statisticsType;
        uint _rosMsgCount;
        std::vector<Duration> _durationHistory{};
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
            // Increment message count
            _statistics.addCount();
        }

    private:
        Statistics _statistics;
    };

} // namespace ros_bridge