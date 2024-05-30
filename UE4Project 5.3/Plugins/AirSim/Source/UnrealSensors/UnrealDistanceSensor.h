// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/distance/DistanceSimple.hpp"
#include "CoordFrameTransformer.h"

class UnrealDistanceSensor : public msr::airlib::DistanceSimple {
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealDistanceSensor(const AirSimSettings::DistanceSetting& setting,
        AActor* actor, const CoordFrameTransformer* ned_transform);

protected:
    virtual msr::airlib::real_T getRayLength(const msr::airlib::Pose& pose) override;

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;


private:
    AActor* actor_;
    const CoordFrameTransformer* ned_transform_;
};