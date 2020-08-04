#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "GameFramework/Actor.h"
#include "ParticleDefinitions.h"

#include <string>
#include "CameraDirector.h"
#include "common/AirSimSettings.hpp"
#include "common/ClockFactory.hpp"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"
#include "Vehicles/Car/CarPawnSimApi.h"
#include "Referee.h"

#include "SimModeBase.generated.h"


UCLASS()
class AIRSIM_API ASimModeBase : public AActor
{
public:

    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Refs")
    ACameraDirector* CameraDirector;


    UFUNCTION(BlueprintCallable, Category = "Api")
    void startApiServer();
    
    UFUNCTION(BlueprintCallable, Category = "Api")
    void stopApiServer();

    UFUNCTION(BlueprintCallable, Category = "Api")
    bool isApiServerStarted();

public:	
    // Sets default values for this actor's properties
    ASimModeBase();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;

    //additional overridable methods
    virtual void reset();
    virtual ECameraDirectorMode getInitialViewMode() const;

    virtual bool isPaused() const;
    virtual void pause(bool is_paused);
    virtual void continueForTime(double seconds);

    virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
        float celestial_clock_speed, float update_interval_secs, bool move_sun);

    const NedTransform& getGlobalNedTransform();

    msr::airlib::ApiProvider* getApiProvider() const
    {
        return api_provider_.get();
    }
    const CarPawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "") const
    {
        return static_cast<CarPawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }
    CarPawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        return static_cast<CarPawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }

    UPROPERTY() UClass* refereeBP_class_;

protected: //must overrides
    typedef msr::airlib::AirSimSettings AirSimSettings;

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const;
    virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const;
    virtual void initializeVehiclePawn(APawn* pawn);
    virtual std::unique_ptr<CarPawnSimApi> createVehicleSimApi(
        const CarPawnSimApi::Params& pawn_sim_api_params) const;
    virtual msr::airlib::VehicleApiBase* getVehicleApi(const CarPawnSimApi::Params& pawn_sim_api_params,
        const CarPawnSimApi* sim_api) const;

protected: //optional overrides
    virtual void setupVehiclesAndCamera();
    virtual void setupInputBindings();
    //called when SimMode should handle clock speed setting
    virtual void setupClockSpeed();
    void initializeCameraDirector(const FTransform& camera_transform, float follow_distance);
    void checkVehicleReady(); //checks if vehicle is available to use

protected: //Utility methods for derived classes
    virtual const msr::airlib::AirSimSettings& getSettings() const;
    FRotator toFRotator(const AirSimSettings::Rotation& rotation, const FRotator& default_val);


protected:
    int record_tick_count;

    UPROPERTY() UClass* pip_camera_class;
private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;

private:
    //assets loaded in constructor
    UPROPERTY() UClass* external_camera_class_;
    UPROPERTY() UClass* camera_director_class_;
    UPROPERTY() UClass* sky_sphere_class_;


    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;
    FRotator default_sun_rotation_;
    TTimePoint tod_sim_clock_start_;             // sim start in local time
    TTimePoint tod_last_update_;
    TTimePoint tod_start_time_;                  // tod, configurable
    bool tod_enabled_;
    float tod_celestial_clock_speed_;
    float tod_update_interval_secs_;
    bool tod_move_sun_;

    std::unique_ptr<NedTransform> global_ned_transform_;
    std::unique_ptr<msr::airlib::WorldSimApiBase> world_sim_api_;
    std::unique_ptr<msr::airlib::ApiProvider> api_provider_;
    std::unique_ptr<msr::airlib::ApiServerBase> api_server_;

    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;

    UPROPERTY()
        TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC

private:
    void setStencilIDs();
    void initializeTimeOfDay();
    void advanceTimeOfDay();
    void setSunRotation(FRotator rotation);
    void showClockStats();
    void drawLidarDebugPoints();
};
