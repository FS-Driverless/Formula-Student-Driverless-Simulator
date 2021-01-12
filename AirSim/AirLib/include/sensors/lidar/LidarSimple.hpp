// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Lidar_hpp
#define msr_airlib_Lidar_hpp

#include <random>
#include "common/Common.hpp"
#include "LidarSimpleParams.hpp"
#include "LidarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class LidarSimple : public LidarBase {
public:
    LidarSimple(const AirSimSettings::LidarSetting& setting = AirSimSettings::LidarSetting())
        : LidarBase(setting.sensor_name)
    {
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
        freq_limiter_.initialize(params_.horizontal_rotation_frequency, 0);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        freq_limiter_.reset();
        last_time_ = clock()->nowNanos();

        updateOutput();
    }

    virtual void update() override
    {
        LidarBase::update();

        freq_limiter_.update();

        if (freq_limiter_.isWaitComplete()) {
            updateOutput();
        }
    }
    
    //*** End: UpdatableState implementation ***//

    virtual ~LidarSimple() = default;

    const LidarSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    virtual void getPointCloud(const Pose& lidar_pose, const Pose& vehicle_pose, 
        TTimeDelta delta_time, vector<real_T>& point_cloud, vector<int>& segmentation_cloud) = 0;

    
private: //methods
    void updateOutput()
    {
        TTimeDelta delta_time = clock()->updateSince(last_time_);

        point_cloud_.clear();

        const GroundTruth& ground_truth = getGroundTruth();

        // we can just add these two poses because they are in the same coordinate frame :)
        Pose lidar_pose = params_.relative_pose + ground_truth.kinematics->pose;
        getPointCloud(params_.relative_pose, // relative lidar pose
            ground_truth.kinematics->pose,   // relative vehicle pose
            delta_time,
            point_cloud_,
            segmentation_cloud_
        );

        LidarData output;
        output.point_cloud = point_cloud_;
        output.time_stamp = clock()->nowNanos();
        output.pose = lidar_pose;            

        last_time_ = output.time_stamp;

        setOutput(output);
        setSegmentationOutput(segmentation_cloud_);
    }

private:
    LidarSimpleParams params_;
    vector<real_T> point_cloud_;
    vector<int> segmentation_cloud_;

    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
};

}} //namespace
#endif
