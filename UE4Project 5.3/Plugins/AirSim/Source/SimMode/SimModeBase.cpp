#include "SimModeBase.h"
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "Runtime/Launch/Resources/Version.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/OutputDeviceNull.h"
#include "Engine/World.h"

#include <memory>
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "common/ScalableClock.hpp"
#include "common/SteppableClock.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "common/EarthCelestial.hpp"
#include "sensors/lidar/LidarSimple.hpp"

#include "Weather/WeatherLib.h"

#include "DrawDebugHelpers.h"

//TODO: this is going to cause circular references which is fine here but
//in future we should consider moving SimMode not derived from AActor and move
//it to AirLib and directly implement WorldSimApiBase interface
#include "WorldSimApi.h"


ASimModeBase::ASimModeBase()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class_val(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class = pip_camera_class_val.Succeeded() ? pip_camera_class_val.Class : nullptr;

    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ = sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;

    static ConstructorHelpers::FClassFinder<AReferee> refereeBP_class(TEXT("Blueprint'/Game/FormulaStudentAssets/RefereeBP'"));
    refereeBP_class_ = refereeBP_class.Succeeded() ? refereeBP_class.Class : nullptr;
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    //get player start
    //this must be done from within actor otherwise we don't get player start
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform player_start_transform = player_controller->GetViewTarget()->GetActorTransform();
    global_ned_transform_.reset(new CoordFrameTransformer(player_start_transform, 
        UAirBlueprintLib::GetWorldToMetersScale(this)));

    world_sim_api_.reset(new WorldSimApi(this));
    api_provider_.reset(new msr::airlib::ApiProvider(world_sim_api_.get()));

    setupClockSpeed();

    setStencilIDs();
    
    record_tick_count = 0;
    setupInputBindings();

    initializeTimeOfDay();
    AirSimSettings::TimeOfDaySetting tod_setting = getSettings().tod_setting;
    setTimeOfDay(tod_setting.enabled, tod_setting.start_datetime, tod_setting.is_start_datetime_dst,
        tod_setting.celestial_clock_speed, tod_setting.update_interval_secs, tod_setting.move_sun);

    setupVehiclesAndCamera();

    UWorld* World = GetWorld();
    if (World)
    {
        UWeatherLib::initWeather(World, spawned_actors_);
        //UWeatherLib::showWeatherMenu(World);
    }
}

const CoordFrameTransformer& ASimModeBase::getGlobalNedTransform()
{
    return *global_ned_transform_;
}

void ASimModeBase::checkVehicleReady()
{
    for (auto& api : api_provider_->getVehicleApis()) {
        if (api) { //sim-only vehicles may have api as null
            std::string message;
            if (!api->isReady(message)) {
                UAirBlueprintLib::LogMessage("Vehicle was not initialized", "", LogDebugLevel::Failure);
                if (message.size() > 0) {
                    UAirBlueprintLib::LogMessage(message.c_str(), "", LogDebugLevel::Failure);
                }
                UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
            }
        }

    }
}

void ASimModeBase::setStencilIDs()
{
    UAirBlueprintLib::SetMeshNamingMethod(getSettings().segmentation_setting.mesh_naming_method);

    if (getSettings().segmentation_setting.init_method ==
        AirSimSettings::SegmentationSetting::InitMethodType::CommonObjectsRandomIDs) {
     
        UAirBlueprintLib::InitializeMeshStencilIDs(!getSettings().segmentation_setting.override_existing);
    }
    //else don't init
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    world_sim_api_.reset();
    api_provider_.reset();
    api_server_.reset();
    global_ned_transform_.reset();

    CameraDirector = nullptr;
    sky_sphere_ = nullptr;
    sun_ = nullptr;

    spawned_actors_.Empty();
    vehicle_sim_apis_.clear();

    Super::EndPlay(EndPlayReason);
}

void ASimModeBase::initializeTimeOfDay()
{
    sky_sphere_ = nullptr;
    sun_ = nullptr;

    TArray<AActor*> sky_spheres;
    UGameplayStatics::GetAllActorsOfClass(this->GetWorld(), sky_sphere_class_, sky_spheres);

    if (sky_spheres.Num() > 1)
        UAirBlueprintLib::LogMessage(TEXT("More than BP_Sky_Sphere were found. "),
            TEXT("TimeOfDay settings would be applied to first one."), LogDebugLevel::Failure);

    if (sky_spheres.Num() >= 1) {
        sky_sphere_ = sky_spheres[0];
        static const FName sun_prop_name(TEXT("Directional light actor"));
        auto* p = sky_sphere_class_->FindPropertyByName(sun_prop_name);
        FObjectProperty* sun_prop = CastField<FObjectProperty>(p);
        UObject* sun_obj = sun_prop->GetObjectPropertyValue_InContainer(sky_sphere_);
        sun_ = Cast<ADirectionalLight>(sun_obj);
        if (sun_)
            default_sun_rotation_ = sun_->GetActorRotation(); 
    }
}

void ASimModeBase::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    bool enabled_currently = tod_enabled_;
    
    if (is_enabled) {

        if (!sun_) {
            UAirBlueprintLib::LogMessage(TEXT("BP_Sky_Sphere was not found. "),
                TEXT("TimeOfDay settings would be ignored."), LogDebugLevel::Failure);
        }
        else {
            sun_->GetRootComponent()->Mobility = EComponentMobility::Movable;

            // this is a bit odd but given how advanceTimeOfDay() works currently, 
            // tod_sim_clock_start_ needs to be reset here.
            tod_sim_clock_start_ = ClockFactory::get()->nowNanos();

            tod_last_update_ = 0;
            if (start_datetime != "")
                tod_start_time_ = Utils::to_time_t(start_datetime, is_start_datetime_dst) * 1E9;
            else
                tod_start_time_ = std::time(nullptr) * 1E9;
        }
    }
    else if (enabled_currently) {
        // Going from enabled to disabled
        if (sun_) {
            setSunRotation(default_sun_rotation_);
            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(ClockFactory::get()->nowNanos() / 1E9), LogDebugLevel::Informational);
        }
    }

    // do these in the end to ensure that advanceTimeOfDay() doesn't see
    // any inconsistent state.
    tod_enabled_ = is_enabled;
    tod_celestial_clock_speed_ = celestial_clock_speed;
    tod_update_interval_secs_ = update_interval_secs;
    tod_move_sun_ = move_sun;
}

bool ASimModeBase::isPaused() const
{
    return false;
}

void ASimModeBase::pause(bool is_paused)
{
    //should be overridden by derived class
    unused(is_paused);
    throw std::domain_error("Pause is not implemented by SimMode");
}

void ASimModeBase::continueForTime(double seconds)
{
    //should be overridden by derived class
    unused(seconds);
    throw std::domain_error("continueForTime is not implemented by SimMode");
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeBase::createApiServer() const
{
    //this will be the case when compilation with RPCLIB is disabled or simmode doesn't support APIs
    return nullptr;
}

void ASimModeBase::setupClockSpeed()
{
    //default setup - this should be overridden in derived modes as needed

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock")
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    else if (clock_type == "SteppableClock")
        ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
            static_cast<msr::airlib::TTimeDelta>(msr::airlib::SteppableClock::DefaultStepSize * clock_speed)));
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    advanceTimeOfDay();

    showClockStats();

    drawLidarDebugPoints();

    Super::Tick(DeltaSeconds);
}

void ASimModeBase::showClockStats()
{
    float clock_speed = getSettings().clock_speed;
    if (clock_speed != 1) {
        UAirBlueprintLib::LogMessageString("ClockSpeed config, actual: ", 
            Utils::stringf("%f, %f", clock_speed, ClockFactory::get()->getTrueScaleWrtWallClock()), 
            LogDebugLevel::Informational);
    }
}

void ASimModeBase::advanceTimeOfDay()
{
    const auto& settings = getSettings();

    if (tod_enabled_ && sky_sphere_ && sun_ && tod_move_sun_) {
        auto secs = ClockFactory::get()->elapsedSince(tod_last_update_);
        if (secs > tod_update_interval_secs_) {
            tod_last_update_ = ClockFactory::get()->nowNanos();

            auto interval = ClockFactory::get()->elapsedSince(tod_sim_clock_start_) * tod_celestial_clock_speed_;
            uint64_t cur_time = ClockFactory::get()->addTo(tod_start_time_, interval)  / 1E9;

            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(cur_time), LogDebugLevel::Informational);

            auto coord = msr::airlib::EarthCelestial::getSunCoordinates(cur_time, settings.origin_geopoint.home_geo_point.latitude,
                settings.origin_geopoint.home_geo_point.longitude);

            setSunRotation(FRotator(-coord.altitude, coord.azimuth, 0));
        }
    }
}

void ASimModeBase::setSunRotation(FRotator rotation)
{
    if (sun_ && sky_sphere_) {
        UAirBlueprintLib::RunCommandOnGameThread([this, rotation]() {
            sun_->SetActorRotation(rotation);

            FOutputDeviceNull ar;
            sky_sphere_->CallFunctionByNameWithArguments(TEXT("UpdateSunDirection"), ar, NULL, true);
        }, true /*wait*/);
    }
}

void ASimModeBase::reset()
{
    //default implementation
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            api->reset();
        }
    }, true);
}

void ASimModeBase::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeBase::reset);
}

ECameraDirectorMode ASimModeBase::getInitialViewMode() const
{
    return Utils::toEnum<ECameraDirectorMode>(getSettings().initial_view_mode);
}

const msr::airlib::AirSimSettings& ASimModeBase::getSettings() const
{
    return AirSimSettings::singleton();
}

void ASimModeBase::initializeCameraDirector(const FTransform& camera_transform, float follow_distance)
{
    TArray<AActor*> camera_dirs;
    UAirBlueprintLib::FindAllActor<ACameraDirector>(this, camera_dirs);
    if (camera_dirs.Num() == 0) {
        //create director
        FActorSpawnParameters camera_spawn_params;
        camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        camera_spawn_params.Name = "CameraDirector";
        CameraDirector = this->GetWorld()->SpawnActor<ACameraDirector>(camera_director_class_, 
            camera_transform, camera_spawn_params);
        CameraDirector->setFollowDistance(follow_distance);
        CameraDirector->setCameraRotationLagEnabled(false);
        //create external camera required for the director
        camera_spawn_params.Name = "ExternalCamera";
        CameraDirector->ExternalCamera = this->GetWorld()->SpawnActor<APIPCamera>(external_camera_class_, 
            camera_transform, camera_spawn_params);
    }
    else {
        CameraDirector = static_cast<ACameraDirector*>(camera_dirs[0]);
    }
}

//API server start/stop
void ASimModeBase::startApiServer()
{
    if (getSettings().enable_rpc) {

#ifdef AIRLIB_NO_RPC
        api_server_.reset();
#else
        api_server_ = createApiServer();
#endif

        try {
            api_server_->start(false, spawned_actors_.Num() + 4);
        }
        catch (std::exception& ex) {
            UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server", ex.what(), LogDebugLevel::Failure);
        }
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);

}
void ASimModeBase::stopApiServer()
{
    if (api_server_ != nullptr) {
        api_server_->stop();
        api_server_.reset(nullptr);
    }
}
bool ASimModeBase::isApiServerStarted()
{
    return api_server_ != nullptr;
}

FRotator ASimModeBase::toFRotator(const msr::airlib::AirSimSettings::Rotation& rotation, const FRotator& default_val)
{
    FRotator frotator = default_val;
    if (!std::isnan(rotation.yaw))
        frotator.Yaw = rotation.yaw;
    if (!std::isnan(rotation.pitch))
        frotator.Pitch = rotation.pitch;
    if (!std::isnan(rotation.roll))
        frotator.Roll = rotation.roll;

    return frotator;
}

void ASimModeBase::setupVehiclesAndCamera()
{
    //get UU origin of global frame
    const FTransform uu_origin = getGlobalNedTransform().getGlobalTransform();

    //determine camera director camera default pose and spawn it
    const auto& camera_director_setting = getSettings().camera_director;
    FVector camera_director_position_uu = uu_origin.GetLocation() + 
        getGlobalNedTransform().fromLocalEnu(camera_director_setting.position);
    FTransform camera_transform(toFRotator(camera_director_setting.rotation, FRotator::ZeroRotator), 
        camera_director_position_uu);
    initializeCameraDirector(camera_transform, camera_director_setting.follow_distance);

    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        getExistingVehiclePawns(pawns);

        APawn* fpv_pawn = nullptr;

        //add vehicles from settings
        for (auto const& vehicle_setting_pair : getSettings().vehicles)
        {
            //if vehicle is of type for derived SimMode and auto creatable
            const auto& vehicle_setting = *vehicle_setting_pair.second;
            if (vehicle_setting.auto_create) {

                //compute initial pose
                FVector spawn_position = uu_origin.GetLocation();
                msr::airlib::Vector3r settings_position = vehicle_setting.position;
                //if (!msr::airlib::VectorMath::hasNan(settings_position))
                    //spawn_position = getGlobalNedTransform().fromGlobalEnu(settings_position);
                FRotator spawn_rotation = toFRotator(vehicle_setting.rotation, uu_origin.Rotator());

                //spawn vehicle pawn
                FActorSpawnParameters pawn_spawn_params;
                pawn_spawn_params.Name = FName(vehicle_setting.vehicle_name.c_str());
                pawn_spawn_params.SpawnCollisionHandlingOverride =
                    ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                    getSettings().pawn_paths.at(getVehiclePawnPathName(vehicle_setting)).pawn_bp);
                APawn* spawned_pawn = static_cast<APawn*>( this->GetWorld()->SpawnActor(
                    vehicle_bp_class, &spawn_position, &spawn_rotation, pawn_spawn_params));

                spawned_actors_.Add(spawned_pawn);
                pawns.Add(spawned_pawn);

                if (vehicle_setting.is_fpv_vehicle)
                    fpv_pawn = spawned_pawn;
            }
        }

        //create API objects for each pawn we have
        for (AActor* pawn : pawns)
        {
            APawn* vehicle_pawn = static_cast<APawn*>(pawn);

            initializeVehiclePawn(vehicle_pawn);

            //create vehicle sim api
            const auto& ned_transform = getGlobalNedTransform();
            const auto& pawn_ned_pos = ned_transform.toLocalEnu(vehicle_pawn->GetActorLocation());
            const auto& home_geopoint= msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
            const std::string vehicle_name = std::string(TCHAR_TO_UTF8(*(vehicle_pawn->GetName())));

            CarPawnSimApi::Params pawn_sim_api_params(vehicle_pawn, &getGlobalNedTransform(),
                getVehiclePawnEvents(vehicle_pawn), getVehiclePawnCameras(vehicle_pawn), pip_camera_class, 
                home_geopoint, vehicle_name);

            auto vehicle_sim_api = createVehicleSimApi(pawn_sim_api_params);
            auto vehicle_sim_api_p = vehicle_sim_api.get();
            auto vehicle_Api = getVehicleApi(pawn_sim_api_params, vehicle_sim_api_p);
            getApiProvider()->insert_or_assign(vehicle_name, vehicle_Api, vehicle_sim_api_p);
            if ((fpv_pawn == vehicle_pawn || !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
                getApiProvider()->makeDefaultVehicle(vehicle_name);

            vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
        }
    }

    if (getApiProvider()->hasDefaultVehicle()) {
        //TODO: better handle no FPV vehicles scenario
        getVehicleSimApi()->possess();
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), getVehicleSimApi()->getPawn(),
            getVehicleSimApi()->getCamera("fpv"), getVehicleSimApi()->getCamera("back_center"), nullptr);
    }
    else
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), nullptr, nullptr, nullptr, nullptr);

    checkVehicleReady();
}

void ASimModeBase::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    //derived class should override this method to retrieve types of pawns they support
}

std::string ASimModeBase::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //derived class should override this method to retrieve types of pawns they support
    return "";
}
PawnEvents* ASimModeBase::getVehiclePawnEvents(APawn* pawn) const
{
    unused(pawn);

    //derived class should override this method to retrieve types of pawns they support
    return nullptr;
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeBase::getVehiclePawnCameras(APawn* pawn) const
{
    unused(pawn);

    //derived class should override this method to retrieve types of pawns they support
    return common_utils::UniqueValueMap<std::string, APIPCamera*>();
}
void ASimModeBase::initializeVehiclePawn(APawn* pawn)
{
    unused(pawn);
    //derived class should override this method to retrieve types of pawns they support
}
std::unique_ptr<CarPawnSimApi> ASimModeBase::createVehicleSimApi(
    const CarPawnSimApi::Params& pawn_sim_api_params) const
{
    unused(pawn_sim_api_params);
    auto sim_api = std::unique_ptr<CarPawnSimApi>();
    sim_api->initialize();

    return sim_api;
}
msr::airlib::VehicleApiBase* ASimModeBase::getVehicleApi(const CarPawnSimApi::Params& pawn_sim_api_params,
    const CarPawnSimApi* sim_api) const
{
    //derived class should override this method to retrieve types of pawns they support
    return nullptr;
}

// Draws debug-points on main viewport for Lidar laser hits.
// Used for debugging only.
void ASimModeBase::drawLidarDebugPoints()
{
    if (getApiProvider() == nullptr)
        return;

    for (auto& sim_api : getApiProvider()->getVehicleSimApis()) {
        CarPawnSimApi* pawn_sim_api = static_cast<CarPawnSimApi*>(sim_api);
        std::string vehicle_name = pawn_sim_api->getVehicleName();

        msr::airlib::VehicleApiBase* api = getApiProvider()->getVehicleApi(vehicle_name);
        if (api != nullptr) {
            
            msr::airlib::uint count_lidars = api->getSensors().size(msr::airlib::SensorBase::SensorType::Lidar);

            for (msr::airlib::uint i = 0; i < count_lidars; i++) {
                // TODO: Is it incorrect to assume LidarSimple here?
                const msr::airlib::LidarSimple* lidar =
                    static_cast<const msr::airlib::LidarSimple*>(api->getSensors().getByType(msr::airlib::SensorBase::SensorType::Lidar, i));
                if (lidar != nullptr && lidar->getParams().draw_debug_points) {

                    msr::airlib::LidarData lidar_data = lidar->getOutput();

                    if (lidar_data.point_cloud.size() < 3)
                        return;

                    for (int j = 0; j < lidar_data.point_cloud.size(); j = j + 3) {
                        msr::airlib::Vector3r point(lidar_data.point_cloud[j], lidar_data.point_cloud[j + 1], lidar_data.point_cloud[j + 2]);
                        msr::airlib::Vector3r point_w = msr::airlib::VectorMath::transformToWorldFrame(point, lidar_data.pose, true);
                        FVector uu_point = pawn_sim_api->getNedTransform().fromLocalEnu(point_w);


                        DrawDebugPoint(
                            this->GetWorld(),
                            uu_point,
                            3,              // size
                            FColor::Green,
                            false,          // persistent (never goes away)
                            0.03            // LifeTime: point leaves a trail on moving object
                        );
                    }
                }
            }
        }
    }
}
