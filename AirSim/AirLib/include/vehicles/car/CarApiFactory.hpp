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
        if (vehicle_setting->vehicle_type == "" || //default config
            vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypePhysXCar) {
            return std::unique_ptr<CarApiBase>(new PhysXCarApi(vehicle_setting, sensor_factory, 
                                                               state, home_geopoint));
        }
        else
            throw std::runtime_error(Utils::stringf(
                    "Cannot create vehicle config because vehicle name '%s' is not recognized",
                    vehicle_setting->vehicle_name.c_str()));
    }
};

}} // namespace

#endif
