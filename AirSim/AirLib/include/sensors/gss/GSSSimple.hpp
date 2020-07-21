#ifndef msr_airlib_GSSSimple_hpp
#define msr_airlib_GSSSimple_hpp


#include "sensors/SensorBase.hpp"


namespace msr { namespace airlib {

class GSSSimple  : public SensorBase {
public:
    // Ground Speed Sensor
    GSSSimple(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public:
    struct Output {
        TTimePoint time_stamp;
        Vector3r linear_velocity;
    };

public:
    virtual void update() override
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        output.time_stamp = clock()->nowNanos();
        output.linear_velocity = Vector3r(std::sqrt(std::pow(ground_truth.kinematics->twist.linear.x(), 2) + std::pow(ground_truth.kinematics->twist.linear.y(), 2)), 0, ground_truth.kinematics->twist.linear.z());

        output_ = output;
    }
    const Output& getOutput() const
    {
        return output_;
    }

    virtual ~GSSSimple() = default;

    virtual void resetImplementation() override {

    }

private:
    Output output_;
};


}}
#endif 
