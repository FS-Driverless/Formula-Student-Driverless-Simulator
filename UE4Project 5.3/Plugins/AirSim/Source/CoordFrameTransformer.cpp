#include "CoordFrameTransformer.h"
#include "AirBlueprintLib.h"

CoordFrameTransformer::CoordFrameTransformer(const FTransform& global_transform, float world_to_meters)
    : CoordFrameTransformer(nullptr, global_transform, world_to_meters)
{
}
CoordFrameTransformer::CoordFrameTransformer(const AActor* pivot, const CoordFrameTransformer& global_transform)
    : CoordFrameTransformer(pivot, global_transform.global_transform_, global_transform.world_to_meters_)
{
}
CoordFrameTransformer::CoordFrameTransformer(const AActor* pivot, const FTransform& global_transform, float world_to_meters)
    : global_transform_(global_transform), world_to_meters_(world_to_meters)
{
    if (pivot != nullptr) {
        //normally pawns have their center as origin. If we use this as 0,0,0 then
        //when we tell vehicle to go to 0,0,0 - it will try to go in the ground
        //so we get the bounds and subtract z to get bottom as 0,0,0
        FVector mesh_origin, mesh_bounds;
        pivot->GetActorBounds(true, mesh_origin, mesh_bounds);

        FVector ground_offset = FVector(0, 0, mesh_bounds.Z);
        local_offset_ = pivot->GetActorLocation() - ground_offset;
    }
    else
        local_offset_ = FVector::ZeroVector;
}

CoordFrameTransformer::Vector3r CoordFrameTransformer::toLocalEnu(const FVector& position) const
{
    return CoordFrameTransformer::toVector3r(position - local_offset_, 1 / world_to_meters_);
}
CoordFrameTransformer::Vector3r CoordFrameTransformer::toLocalEnuVelocity(const FVector& velocity) const
{
    return CoordFrameTransformer::toVector3r(velocity, 1 / world_to_meters_);
}
CoordFrameTransformer::Vector3r CoordFrameTransformer::toGlobalEnu(const FVector& position) const
{
    return CoordFrameTransformer::toVector3r(position - global_transform_.GetLocation(), 1 / world_to_meters_);
}
CoordFrameTransformer::Quaternionr CoordFrameTransformer::toEnu(const FQuat& q) const
{   
    auto q1 = Quaternionr(q.W, q.X, q.Y, q.Z);

    return msr::airlib::VectorMath::toQuaternion(msr::airlib::VectorMath::getPitch(q1), - msr::airlib::VectorMath::getRoll(q1), - msr::airlib::VectorMath::getYaw(q1));
}
float CoordFrameTransformer::toEnu(float length) const
{
    return length / world_to_meters_;
}
CoordFrameTransformer::Pose CoordFrameTransformer::toLocalEnu(const FTransform& pose) const
{
    return Pose(toLocalEnu(pose.GetLocation()), toEnu(pose.GetRotation()));
}
CoordFrameTransformer::Pose CoordFrameTransformer::toGlobalEnu(const FTransform& pose) const
{
    return Pose(toGlobalEnu(pose.GetLocation()), toEnu(pose.GetRotation()));
}

float CoordFrameTransformer::fromEnu(float length) const
{
    return length * world_to_meters_;
}
FVector CoordFrameTransformer::fromLocalEnu(const CoordFrameTransformer::Vector3r& position) const
{
    return CoordFrameTransformer::toFVector(position, world_to_meters_) + local_offset_;
}
FVector CoordFrameTransformer::fromGlobalEnu(const CoordFrameTransformer::Vector3r& position) const
{
    return CoordFrameTransformer::toFVector(position, world_to_meters_) + global_transform_.GetLocation();
}
FQuat CoordFrameTransformer::fromEnu(const Quaternionr& q) const
{    
    // inverse of toEnu
    auto rot = msr::airlib::VectorMath::toQuaternion(msr::airlib::VectorMath::getPitch(q), - msr::airlib::VectorMath::getRoll(q), - msr::airlib::VectorMath::getYaw(q));
    return FQuat(rot.x(), rot.y(), rot.z(), rot.w());
}
FTransform CoordFrameTransformer::fromLocalEnu(const Pose& pose) const
{
    return FTransform(fromEnu(pose.orientation), fromLocalEnu(pose.position));
}

FRotator CoordFrameTransformer::fromEnu(float pitch, float yaw, float roll) const
{
    // Relative coordinate frame is right handed
    // UU roll and pitch are right handed, yaw is left handed
    // So we need to invert yaw.

    // In relative coordinates: x=forward, Y=left and Z=up. 
    // UU has X=forward, Y=right, Z=up
    // So the Y axis is inverted, so we need to invert pitch.
    return FRotator(pitch, -yaw, -roll);
}

FTransform CoordFrameTransformer::fromGlobalEnu(const Pose& pose) const
{
    return FTransform(fromEnu(pose.orientation), fromGlobalEnu(pose.position));
}

FVector CoordFrameTransformer::getGlobalOffset() const
{
    return global_transform_.GetLocation();
}
FVector CoordFrameTransformer::getLocalOffset() const
{
    return local_offset_;
}
FTransform CoordFrameTransformer::getGlobalTransform() const
{
    return global_transform_;
}

FVector CoordFrameTransformer::toFVector(const Vector3r& vec, float scale) const
{
    return FVector(vec.x() * scale, - vec.y() * scale, vec.z() * scale);
}

CoordFrameTransformer::Vector3r CoordFrameTransformer::toVector3r(const FVector& vec, float scale) const
{
    return Vector3r(vec.X * scale, - vec.Y * scale, vec.Z * scale);
}
