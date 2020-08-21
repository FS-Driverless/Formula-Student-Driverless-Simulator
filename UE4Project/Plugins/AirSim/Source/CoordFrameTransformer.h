#pragma once

#include "CoreMinimal.h"
#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Actor.h"
#include "common/Common.hpp"

/*
    Note on coordinate system
    -------------------------
    We have following coordinate systems:
    (1) UU or Unreal Units or Unreal Coordinate system. This is X=North, Y=East, Z=Up.
    (2) Global: This is transformation of UU with origin set to 0,0,0. This is ENU X=East, Y=North, Z=Up
    (3) Local: This is transformation of UU with origin set to vehicle's spawning UU location
*/

class AIRSIM_API CoordFrameTransformer
{
public:
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::Pose Pose;

public:
    CoordFrameTransformer(const FTransform& global_transform, float world_to_meters);
    CoordFrameTransformer(const AActor* pivot, const CoordFrameTransformer& global_transform);

    //UU -> local ENU
    Vector3r toLocalEnu(const FVector& position) const;
    Vector3r toLocalEnuVelocity(const FVector& velocity) const;
    Vector3r toGlobalEnu(const FVector& position) const;
    Quaternionr toEnu(const FQuat& q) const;
    float toEnu(float length) const;
    Pose toLocalEnu(const FTransform& pose) const;
    Pose toGlobalEnu(const FTransform& pose) const;

    //local ENU -> UU
    FVector fromLocalEnu(const Vector3r& position) const;
    FVector fromGlobalEnu(const Vector3r& position) const;
    FQuat fromEnu(const Quaternionr& q) const;
    float fromEnu(float length) const;
    FTransform fromLocalEnu(const Pose& pose) const;
    FTransform fromGlobalEnu(const Pose& pose) const;

    FVector getGlobalOffset() const;
    FVector getLocalOffset() const;
    FTransform getGlobalTransform() const;

private:
    CoordFrameTransformer(const AActor* pivot, const FTransform& global_transform, float world_to_meters); //create only through static factory methods
    FVector toFVector(const Vector3r& vec, float scale) const;
    Vector3r toVector3r(const FVector& vec, float scale) const;

private:
    FTransform global_transform_;
    float world_to_meters_;
    FVector local_offset_;
};
