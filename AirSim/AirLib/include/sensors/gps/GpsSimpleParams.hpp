// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpsSimpleParams_hpp
#define msr_airlib_GpsSimpleParams_hpp

#include "common/Common.hpp"


namespace msr { namespace airlib {

struct GpsSimpleParams {

    // we want a constant eph and epv so time constant set to 1
    real_T eph_time_constant = 1, epv_time_constant = 1;

    // from the start we want to have the final resoltuions. Thats why initial and final are equal.
    real_T eph_initial = 0.04f, epv_initial = 0.04f;   
    real_T eph_final = 0.04f, epv_final = 0.04f;

    real_T update_latency = 0;    //sec
    real_T update_frequency = 10;    //Hz
    real_T startup_delay = 0;        //sec

    void initializeFromSettings(const AirSimSettings::GpsSetting& settings)
    {
        unused(settings);
    }
};

}} //namespace
#endif
