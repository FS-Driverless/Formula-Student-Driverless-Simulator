// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

class CarApiBase : public VehicleApiBase
{
public:
    struct CarControls
    {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
                    bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
              is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        void set_throttle(float throttle_val, bool forward)
        {
            if (forward)
            {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            else
            {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = -std::abs(throttle_val);
            }
        }
    };

    struct CarState
    {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;

        CarState()
        {
        }

        CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val,
                 const Kinematics::State &kinematics_estimated_val, uint64_t timestamp_val)
            : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val),
              kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }
        //shortcuts
        const Vector3r &getPosition() const
        {
            return kinematics_estimated.pose.position;
        }
        const Quaternionr &getOrientation() const
        {
            return kinematics_estimated.pose.orientation;
        }
    };

    // Some utility structs for passing around cones and positions

    enum ConeColor {
        Yellow,
        Blue,
        OrangeLarge,
        OrangeSmall,
        Unknown
    };

    struct Point2D {
        float x;
        float y;
    };

    struct Cone {
        Point2D location;
        ConeColor color;
    };

    struct RefereeState {
        int doo_counter = 0;
	    std::vector<float> laps;
        std::vector<Cone> cones;
        Point2D car_start_location;

        RefereeState() 
        {
        }

        RefereeState(const int &doo_counter_val, const std::vector<float> &laps_val, const std::vector<Cone> &cones, const Point2D &init_pos) 
        : doo_counter(doo_counter_val), laps(laps_val), cones(cones), car_start_location(init_pos)
        {
        }


    };

public:
    // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
    //CarApiBase() {}

    CarApiBase(const AirSimSettings::VehicleSetting *vehicle_setting,
               std::shared_ptr<SensorFactory> sensor_factory,
               const Kinematics::State &state)
    {
        initialize(vehicle_setting, sensor_factory, state);
    }

    virtual void update() override
    {
        VehicleApiBase::update();

        getSensors().update();
    }

    // sensor helpers
    virtual const SensorCollection &getSensors() const override
    {
        return sensors_;
    }

    SensorCollection &getSensors()
    {
        return sensors_;
    }

    void initialize(const AirSimSettings::VehicleSetting *vehicle_setting,
                    std::shared_ptr<SensorFactory> sensor_factory,
                    const Kinematics::State &state)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();

        addSensorsFromSettings(vehicle_setting);

        getSensors().initialize(&state);
    }

    void addSensorsFromSettings(const AirSimSettings::VehicleSetting *vehicle_setting)
    {
        // use sensors from vehicle settings; if empty list, use default sensors.
        // note that the vehicle settings completely override the default sensor "list";
        // there is no piecemeal add/remove/update per sensor.
        const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>> &sensor_settings = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;

        sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
    }

    virtual void setCarControls(const CarControls &controls) = 0;
    virtual void updateCarState(const CarState &state) = 0;
    virtual const CarState &getCarState() const = 0;
    virtual const CarControls &getCarControls() const = 0;

    virtual ~CarApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_;                      //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors

protected:
    virtual void resetImplementation() override
    {
        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }
};

} // namespace airlib
} // namespace msr
#endif
