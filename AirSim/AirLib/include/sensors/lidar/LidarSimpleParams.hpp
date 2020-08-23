// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarSimpleParams_hpp
#define msr_airlib_LidarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct LidarSimpleParams {

    // default values
    uint number_of_lasers = 16; 
    real_T range = 10000.0f / 100;            // meters
    uint points_per_scan = 100000;  
    uint horizontal_rotation_frequency = 10;  // rotations/sec
    real_T horizontal_FOV_start = 0;
    real_T horizontal_FOV_end = 359;
    real_T vertical_FOV_upper = -15;
    real_T vertical_FOV_lower = -45;

    Pose relative_pose {
        Vector3r(0, 0, 1),                     // position - a little above vehicle
        Quaternionr::Identity()               // orientation - by default Quaternionr(1, 0, 0, 0) 
    };        

    bool draw_debug_points = false;

    void initializeFromSettings(const AirSimSettings::LidarSetting& settings)
    {
        number_of_lasers = settings.number_of_lasers;
        range = settings.range;
        points_per_scan = settings.points_per_scan;
        horizontal_rotation_frequency = settings.horizontal_rotation_frequency;

        horizontal_FOV_start = settings.horizontal_FOV_start;
        horizontal_FOV_end = settings.horizontal_FOV_end;

        // By default, for multirotors the lidars FOV point downwards;
        // for cars, the lidars FOV is more forward facing.
        vertical_FOV_upper = settings.vertical_FOV_upper;
        if (std::isnan(vertical_FOV_upper)) {
            vertical_FOV_upper = +10;
        }

        vertical_FOV_lower = settings.vertical_FOV_lower;
        if (std::isnan(vertical_FOV_lower)) {
            vertical_FOV_lower = -10;
        }

        relative_pose.position = settings.position;
        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z())) {
            relative_pose.position.z() = 1;  // a little bit above for cars
        }

        float pitch, roll, yaw;
        pitch = !std::isnan(settings.rotation.pitch) ? settings.rotation.pitch : 0;
        roll = !std::isnan(settings.rotation.roll) ? settings.rotation.roll : 0;
        yaw = !std::isnan(settings.rotation.yaw) ? settings.rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch),   //pitch - rotation around Y axis
            Utils::degreesToRadians(roll),    //roll  - rotation around X axis
            Utils::degreesToRadians(yaw));    //yaw   - rotation around Z axis
           
        draw_debug_points = settings.draw_debug_points;
    }
};

}} //namespace
#endif
