// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_CarApiFactory_hpp
#define msr_airlib_vehicles_CarApiFactory_hpp

#include "vehicles/car/firmwares/physxcar/PhysXCarApi.hpp"

namespace msr { namespace airlib {

class CarApiFactory {
public:
    static std::unique_ptr<CarApiBase> createApi(const AirSimSettings::VehicleSetting* vehicle_setting, 
                                                 std::shared_ptr<SensorFactory> sensor_factory, 
                                                 const Kinematics::State& state, 
                                                 const msr::airlib::GeoPoint& home_geopoint)
    {
        return std::unique_ptr<CarApiBase>(new PhysXCarApi(vehicle_setting, sensor_factory, state, home_geopoint));
    }
};

}} // namespace

#endif
