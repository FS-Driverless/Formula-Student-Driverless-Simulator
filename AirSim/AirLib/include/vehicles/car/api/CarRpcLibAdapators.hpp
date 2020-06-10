// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibAdapators_hpp
#define air_CarRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

MSGPACK_ADD_ENUM(msr::airlib::CarApiBase::ConeColor);

namespace msr { namespace airlib_rpclib {

class CarRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        float brake = 0;
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate);

        CarControls()
        {}

        CarControls(const msr::airlib::CarApiBase::CarControls& s)
        {
            throttle = s.throttle;
            steering = s.steering;
            brake = s.brake;
            handbrake = s.handbrake;
            is_manual_gear = s.is_manual_gear;
            manual_gear = s.manual_gear;
            gear_immediate = s.gear_immediate;
        }
        msr::airlib::CarApiBase::CarControls to() const
        {
            return msr::airlib::CarApiBase::CarControls(throttle, steering, brake, handbrake,
                is_manual_gear, manual_gear, gear_immediate);
        }
    };

    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        KinematicsState kinematics_estimated;
        uint64_t timestamp;

        MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, timestamp);

        CarState()
        {}

        CarState(const msr::airlib::CarApiBase::CarState& s)
        {
            speed = s.speed;
            gear = s.gear;
            rpm = s.rpm;
            maxrpm = s.maxrpm;
            handbrake = s.handbrake;
            timestamp = s.timestamp;
            kinematics_estimated = s.kinematics_estimated;
        }
        msr::airlib::CarApiBase::CarState to() const
        {
            return msr::airlib::CarApiBase::CarState(
                speed, gear, rpm, maxrpm, handbrake, kinematics_estimated.to(), timestamp);
        }
    };

    struct Cone {
        float x;
        float y;
        msr::airlib::CarApiBase::ConeColor color;
        MSGPACK_DEFINE_MAP(x, y, color);
    };

    struct Point2D {
        float x;
        float y;
        MSGPACK_DEFINE_MAP(x, y);
    };

    struct RefereeState {
        int doo_counter = 0;
	    std::vector<float> laps;
        std::vector<Cone> cones;
        Point2D initial_position;

        MSGPACK_DEFINE_MAP(doo_counter, laps, cones, initial_position);

        RefereeState() {}
        RefereeState(const msr::airlib::CarApiBase::RefereeState& s) {
            doo_counter = s.doo_counter;
	        laps = s.laps;
            initial_position.x = s.car_start_location.x;
            initial_position.y = s.car_start_location.y;

            for (size_t i = 0; i < s.cones.size(); i++) {
                Cone cone;
                cone.x = s.cones[i].location.x;
                cone.y = s.cones[i].location.y;
                cone.color = s.cones[i].color;
                cones.push_back(cone);
            }
        }

        msr::airlib::CarApiBase::RefereeState to() const
        {
            msr::airlib::CarApiBase::Point2D initial_position_api_base;
            initial_position_api_base.x = initial_position.x;
            initial_position_api_base.y = initial_position.y;
            std::vector<msr::airlib::CarApiBase::Cone> cones_api_base;
            for (size_t i = 0; i < cones.size(); i++) {
                msr::airlib::CarApiBase::Cone cone_api_base;
                cone_api_base.location.x = cones[i].x;
                cone_api_base.location.y = cones[i].y;
                cone_api_base.color = cones[i].color;
                cones_api_base.push_back(cone_api_base);
            }
            return msr::airlib::CarApiBase::RefereeState(doo_counter, laps, cones_api_base, initial_position_api_base);
        }

    };
};

}} //namespace


#endif
