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
        //normally pawns have their center as origin. If we use this as 0,0,0 in NED then
        //when we tell vehicle to go to 0,0,0 - it will try to go in the ground
        //so we get the bounds and subtract z to get bottom as 0,0,0
        FVector mesh_origin, mesh_bounds;
        pivot->GetActorBounds(true, mesh_origin, mesh_bounds);

        FVector ground_offset = FVector(0, 0, mesh_bounds.Z);
        local_ned_offset_ = pivot->GetActorLocation() - ground_offset;
    }
    else
        local_ned_offset_ = FVector::ZeroVector;
}

CoordFrameTransformer::Vector3r CoordFrameTransformer::toLocalNed(const FVector& position) const
{
    return CoordFrameTransformer::toVector3r(position - local_ned_offset_, 
        1 / world_to_meters_, true);
}
CoordFrameTransformer::Vector3r CoordFrameTransformer::toLocalNedVelocity(const FVector& velocity) const
{
    return CoordFrameTransformer::toVector3r(velocity,
        1 / world_to_meters_, true);
}
CoordFrameTransformer::Vector3r CoordFrameTransformer::toGlobalNed(const FVector& position) const
{
    return CoordFrameTransformer::toVector3r(position - global_transform_.GetLocation(),
        1 / world_to_meters_, true);
}
CoordFrameTransformer::Quaternionr CoordFrameTransformer::toNed(const FQuat& q) const
{
    return Quaternionr(q.W, -q.X, -q.Y, q.Z);
}
float CoordFrameTransformer::toNed(float length) const
{
    return length / world_to_meters_;
}
CoordFrameTransformer::Pose CoordFrameTransformer::toLocalNed(const FTransform& pose) const
{
    return Pose(toLocalNed(pose.GetLocation()), toNed(pose.GetRotation()));
}
CoordFrameTransformer::Pose CoordFrameTransformer::toGlobalNed(const FTransform& pose) const
{
    return Pose(toGlobalNed(pose.GetLocation()), toNed(pose.GetRotation()));
}

float CoordFrameTransformer::fromNed(float length) const
{
    return length * world_to_meters_;
}
FVector CoordFrameTransformer::fromLocalNed(const CoordFrameTransformer::Vector3r& position) const
{
    return CoordFrameTransformer::toFVector(position, world_to_meters_, true) + local_ned_offset_;
}
FVector CoordFrameTransformer::fromGlobalNed(const CoordFrameTransformer::Vector3r& position) const
{
    return CoordFrameTransformer::toFVector(position, world_to_meters_, true) + global_transform_.GetLocation();
}
FQuat CoordFrameTransformer::fromNed(const Quaternionr& q) const
{
    return FQuat(-q.x(), -q.y(), q.z(), q.w());
}
FTransform CoordFrameTransformer::fromLocalNed(const Pose& pose) const
{
    return FTransform(fromNed(pose.orientation), fromLocalNed(pose.position));
}
FTransform CoordFrameTransformer::fromGlobalNed(const Pose& pose) const
{
    return FTransform(fromNed(pose.orientation), fromGlobalNed(pose.position));
}
FQuat CoordFrameTransformer::fromUUtoFLU(const FQuat& q) const
{
    return FQuat(-q.X, q.Y, -q.Z, q.W);
}

FVector CoordFrameTransformer::getGlobalOffset() const
{
    return global_transform_.GetLocation();
}
FVector CoordFrameTransformer::getLocalOffset() const
{
    return local_ned_offset_;
}
FTransform CoordFrameTransformer::getGlobalTransform() const
{
    return global_transform_;
}

FVector CoordFrameTransformer::toFVector(const Vector3r& vec, float scale, bool convert_from_ned) const
{
    return FVector(vec.x() * scale, vec.y() * scale, 
        (convert_from_ned ? -vec.z() : vec.z()) * scale);
}

CoordFrameTransformer::Vector3r CoordFrameTransformer::toVector3r(const FVector& vec, float scale, bool convert_to_ned) const
{
    return Vector3r(vec.X * scale, vec.Y * scale,
        (convert_to_ned ? -vec.Z : vec.Z)  * scale);
}
