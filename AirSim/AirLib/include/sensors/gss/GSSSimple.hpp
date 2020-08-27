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
        SensorBase::update();
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        output.time_stamp = clock()->nowNanos();
        output.linear_velocity = ground_truth.kinematics->twist.linear;

        output_ = output;
    }
    const Output& getOutput() const
    {
        return output_;
    }

    virtual ~GSSSimple() = default;

    virtual void resetImplementation() override {
        update();
    }

private:
    Output output_;
};


}}
#endif 
