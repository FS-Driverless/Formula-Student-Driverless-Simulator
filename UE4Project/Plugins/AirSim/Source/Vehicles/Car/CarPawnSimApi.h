#pragma once

#include "GameFramework/Pawn.h"
#include "Particles/ParticleSystemComponent.h"
#include "UnrealImageCapture.h"

#include <vector>
#include <memory>
#include "PIPCamera.h"
#include "CoordFrameTransformer.h"
#include "common/AirSimSettings.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CarPawn.h"
#include "PawnEvents.h"
#include "CarPawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "vehicles/car/firmwares/physxcar/PhysXCarApi.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

class CarPawnSimApi : public msr::airlib::VehicleSimApiBase
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;

    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::CollisionInfo CollisionInfo;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    struct Params {
        APawn* pawn; 
        const CoordFrameTransformer* global_transform;
        PawnEvents* pawn_events;
        common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
        UClass* pip_camera_class;
        msr::airlib::GeoPoint home_geopoint;
        std::string vehicle_name;

        Params()
        {
        }

        Params(APawn* pawn_val, const CoordFrameTransformer* global_transform_val, PawnEvents* pawn_events_val,
            const common_utils::UniqueValueMap<std::string, APIPCamera*> cameras_val, UClass* pip_camera_class_val,
            const msr::airlib::GeoPoint home_geopoint_val,
            std::string vehicle_name_val)
        {
            pawn = pawn_val; 
            global_transform = global_transform_val;
            pawn_events = pawn_events_val;
            cameras = cameras_val;
            pip_camera_class = pip_camera_class_val;
            home_geopoint = home_geopoint_val;
            vehicle_name = vehicle_name_val;
        }
    };

public:
    virtual void initialize() override;
    virtual ~CarPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    CarPawnSimApi(const Params& params,
        const msr::airlib::CarApiBase::CarControls& keyboard_controls, UWheeledVehicleMovementComponent* movement);

    virtual void update() override;
    void updatePawn();

    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    msr::airlib::CarApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const
    {
        return vehicle_api_.get();
    }

    virtual const UnrealImageCapture* getImageCapture() const override;
    virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& request) const override;
    virtual std::vector<uint8_t> getImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type) const override;
    virtual Pose getPose() const override;
    virtual void setPose(const Pose& pose, bool ignore_collision) override;

    virtual msr::airlib::CameraInfo getCameraInfo(const std::string& camera_name) const override;
    const APIPCamera* getCamera(const std::string& camera_name) const;
    APIPCamera* getCamera(const std::string& camera_name);
    int getCameraCount();

    APawn* getPawn();
    msr::airlib::CarApiBase::CarState getPawnCarState() const;

    FVector getUUPosition() const;
    FRotator getUUOrientation() const;

    const CoordFrameTransformer& getNedTransform() const;
    void possess();
    void setRCForceFeedback(float rumble_strength, float auto_center);


    virtual void setCameraOrientation(const std::string& camera_name, const Quaternionr& orientation) override;
    virtual void setCameraFoV(const std::string& camera_name, float fov_degrees) override;
    virtual CollisionInfo getCollisionInfo() const override;
    virtual int getRemoteControlID() const override;
    virtual msr::airlib::RCData getRCData() const override;
    virtual std::string getVehicleName() const override
    {
        return params_.vehicle_name;
    }
    virtual void toggleTrace() override;
    virtual void setTraceLine(const std::vector<float>& color_rgba, float thickness) override;

    virtual const msr::airlib::Kinematics::State* getGroundTruthKinematics() const override;
    virtual const msr::airlib::WheelStates* getWheelStates() const override;

    void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);


protected:
    virtual void resetImplementation() override;
    void resetPawn();
    msr::airlib::Kinematics* getKinematics();
    virtual void pawnTick(float dt);
    void setPoseInternal(const Pose& pose, bool ignore_collision);

private:
    void createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
    void updateCarControls();

    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();
    void detectUsbRc();
    void setupCamerasFromSettings(const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras);
    void createCamerasFromSettings();
    //on collision, pawns should update this
    void onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp,
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit);

    //these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    CarPawnSimApi::Pose toPose(const FVector& u_position, const FQuat& u_quat) const;
    void updateKinematics(float dt);
    void updateWheelStates(float dt);
    void setStartPosition(const FVector& position, const FRotator& rotator);
private:
    Params params_;

    std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
    ACarPawn* pawn_;
    UWheeledVehicleMovementComponent* movement_;
    msr::airlib::CarApiBase::CarControls last_controls_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    std::vector<std::string> vehicle_api_messages_;

    //storing reference from pawn
    const msr::airlib::CarApiBase::CarControls& keyboard_controls_;

    msr::airlib::CarApiBase::CarControls joystick_controls_;
    msr::airlib::CarApiBase::CarControls current_controls_;

    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef msr::airlib::Kinematics Kinematics;

    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras_;
    msr::airlib::GeoPoint home_geo_point_;

    std::string vehicle_name_;
    CoordFrameTransformer ned_transform_;

    FVector ground_trace_end_;
    FVector ground_margin_;
    std::unique_ptr<UnrealImageCapture> image_capture_;
    std::string log_line_;

    mutable msr::airlib::RCData rc_data_;
    mutable SimJoyStick joystick_;
    mutable SimJoyStick::State joystick_state_;

    struct State {
        FVector start_location;
        FRotator start_rotation;
        FVector last_position;
        FVector last_debug_position;
        FVector current_position;
        FVector current_debug_position;
        FVector debug_position_offset;        
        bool tracing_enabled;
        bool collisions_enabled;
        bool passthrough_enabled;
        bool was_last_move_teleport;
        CollisionInfo collision_info;

        FVector mesh_origin;
        FVector mesh_bounds;
        FVector ground_offset;
        FVector transformation_offset;
    };
    
    State state_, initial_state_;

    std::unique_ptr<msr::airlib::Kinematics> kinematics_;
    std::unique_ptr<msr::airlib::WheelStates> wheel_states_;

    FColor trace_color_ = FColor::Purple;
    float trace_thickness_ = 3.0f;
};
