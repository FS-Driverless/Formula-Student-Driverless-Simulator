// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_DistanceBase_hpp
#define msr_airlib_DistanceBase_hpp


#include "sensors/SensorBase.hpp"


namespace msr { namespace airlib {

class DistanceBase  : public SensorBase {
public:
    DistanceBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    struct Output { //same fields as ROS message
        TTimePoint time_stamp;
        real_T distance;    //meters
        real_T min_distance;//m
        real_T max_distance;//m
        Pose relative_pose;
    };


public:

    const Output& getOutput() const
    {
        return output_;
    }

protected:
    void setOutput(const Output& output)
    {
        output_ = output;
    }


private:
    Output output_;
};


}} //namespace
#endif
