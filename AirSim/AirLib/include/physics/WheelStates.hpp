#ifndef airsim_core_WheelStates_hpp
#define airsim_core_WheelStates_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {

struct WheelState {
    float rotation_angle;
    float rpm;
    float steering_angle;
};

struct WheelStates {
    msr::airlib::TTimePoint time_stamp = 0;
    
    WheelState fl;
    WheelState fr;
    WheelState rl;
    WheelState rr;
};

}}

#endif
