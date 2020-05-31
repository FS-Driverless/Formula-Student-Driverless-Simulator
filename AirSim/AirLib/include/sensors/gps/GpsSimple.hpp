// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Gps_hpp
#define msr_airlib_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"
#include "common/EarthUtils.hpp"
#include "math.h"

namespace msr { namespace airlib {

class GpsSimple : public GpsBase {
public: //methods
    GpsSimple(const AirSimSettings::GpsSetting& setting = AirSimSettings::GpsSetting())
        : GpsBase(setting.sensor_name)
    {
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
        freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
        delay_line_.initialize(params_.update_latency);

        //initialize filters
        eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targeted 0.3f, with 45% accuracy within 100 updates, each update occurring at 0.2s interval
        epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);
        //get zero velocity offset
        zero_vel_offset=getZeroVelOffset(params_.eph_final, params_.epv_final);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        freq_limiter_.reset();
        delay_line_.reset();

        eph_filter.reset();
        epv_filter.reset();

        addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
    }

    virtual void update() override
    {
        GpsBase::update();

        freq_limiter_.update();
        eph_filter.update();
        epv_filter.update();

        if (freq_limiter_.isWaitComplete()) {   //update output
            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
        }
        delay_line_.update();

        if (freq_limiter_.isWaitComplete())
            setOutput(delay_line_.getOutput());
    }

    //*** End: UpdatableState implementation ***//

    virtual ~GpsSimple() = default;
    //generate Gaussian noise 
    const float getGaussianNoise(float mean, float var)
    {
        std::normal_distribution<float> distribution(mean, var);
        auto seed = (unsigned int)std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        return distribution(generator);
    }
    //Generate noisy geo_point for moving vehicle
    msr::airlib::GeoPoint generateErrors(msr::airlib::GeoPoint geo_point, real_T eph, real_T epv)
    {
        HomeGeoPoint geo_point_in = HomeGeoPoint(geo_point);
        float var_h = pow(eph, 2)/18;
        float var_v = pow(epv, 2)/9;
        msr::airlib::GeoPoint geo_point_err = EarthUtils::nedToGeodetic(msr::airlib::Vector3r(getGaussianNoise(0, var_h), getGaussianNoise(0,var_h), getGaussianNoise(0, var_v)), geo_point_in);
        msr::airlib::GeoPoint geo_point_out = geo_point;
        geo_point_out.latitude = geo_point_err.latitude;
        geo_point_out.longitude = geo_point_err.longitude;
        geo_point_out.altitude = geo_point_err.altitude;

        return geo_point_out;
    }

    //Create a constant offset to add to the geo_point when vehicle at a standstill
    msr::airlib::Vector3r getZeroVelOffset(real_T eph_final, real_T epv_final)
    {   float var_h = pow(eph_final, 2)/18;
        float var_v = pow(epv_final, 2)/9;
        return msr::airlib::Vector3r(getGaussianNoise(0, var_h), getGaussianNoise(0, var_h), getGaussianNoise(0, var_v));
    }

private:
    void addOutputToDelayLine(real_T eph, real_T epv)
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();
        
        //GNSS
        output.gnss.time_utc = static_cast<uint64_t>(clock()->nowNanos() / 1.0E3);
        output.gnss.velocity = ground_truth.kinematics->twist.linear;
        //add noise to the GPS position measurements depending on the velocity of the vehicle
        if ((gpsnoise) && (sqrt(pow(output.gnss.velocity.x(), 2) + pow(output.gnss.velocity.y(), 2) + pow(output.gnss.velocity.z(), 2))>0.5)) {
            output.gnss.geo_point=generateErrors(ground_truth.environment->getState().geo_point, eph, epv);
        }
        else {
            output.gnss.geo_point=EarthUtils::nedToGeodetic(zero_vel_offset, ground_truth.environment->getState().geo_point);
        }
        output.gnss.eph = eph;
        output.is_valid = true;
        output.gnss.epv = epv;

        output.time_stamp = clock()->nowNanos();

        delay_line_.push_back(output);
    }


private:
    typedef std::normal_distribution<> NormalDistribution;
    bool gpsnoise=true;
    msr::airlib::Vector3r zero_vel_offset;
    GpsSimpleParams params_;
    FirstOrderFilter<real_T> eph_filter, epv_filter;
    FrequencyLimiter freq_limiter_;
    DelayLine<Output> delay_line_;
};

}} //namespace
#endif