// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ImuBase_hpp
#define msr_airlib_ImuBase_hpp


#include "sensors/SensorBase.hpp"
#include "common/Common.hpp"


namespace msr { namespace airlib {

class ImuBase  : public SensorBase {
public:
    ImuBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public: //types
    struct Output {	//structure is same as ROS IMU message
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        TTimePoint time_stamp; 
        Quaternionr orientation;
        Vector3r angular_velocity;
        real_T sigma_arw, sigma_vrw;
        Vector3r linear_acceleration;
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
