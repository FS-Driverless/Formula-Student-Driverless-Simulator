// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "sensors/SensorFactory.hpp"
#include <memory>
#include "CoordFrameTransformer.h"
#include "GameFramework/Actor.h"

class UnrealSensorFactory : public msr::airlib::SensorFactory {
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealSensorFactory(AActor* actor, const CoordFrameTransformer* ned_transform);
    virtual ~UnrealSensorFactory() {}
    void setActor(AActor* actor, const CoordFrameTransformer* ned_transform);
    virtual std::unique_ptr<msr::airlib::SensorBase> createSensorFromSettings(
        const AirSimSettings::SensorSetting* sensor_setting) const override;

private:
    AActor* actor_;
    const CoordFrameTransformer* ned_transform_;
};
