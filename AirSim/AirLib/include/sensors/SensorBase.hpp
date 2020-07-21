// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SensorBase_hpp
#define msr_airlib_SensorBase_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"


namespace msr { namespace airlib {

/*
    Derived classes should not do any work in constructor which requires ground truth.
    After construction of the derived class an initialize(...) must be made which would
    set the sensor in good-to-use state by call to reset.
*/
class SensorBase : public UpdatableObject  {
public:
    enum class SensorType : uint {
        Imu = 2,
        Gps = 3,
        Distance = 5,
        Lidar = 6
    };

    SensorBase(const std::string& sensor_name = "")
        : name_(sensor_name)
    {}

protected:
    struct GroundTruth {
        const Kinematics::State* kinematics;
    };
public:
    virtual void initialize(const Kinematics::State* kinematics)
    {
        ground_truth_.kinematics = kinematics;
    }

    const GroundTruth& getGroundTruth() const
    {
        return ground_truth_;
    }
   
    const std::string& getName() const
    {
        return name_;
    }

    virtual ~SensorBase() = default;

private:
    //ground truth can be shared between many sensors
    GroundTruth ground_truth_;
    std::string name_ = "";
};



}} //namespace
#endif
