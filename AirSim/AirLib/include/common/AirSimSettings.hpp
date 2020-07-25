// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_AirSimSettings_hpp
#define airsim_core_AirSimSettings_hpp

#include <string>
#include <vector>
#include <exception>
#include <functional>
#include <map>
#include "Settings.hpp"
#include "CommonStructs.hpp"
#include "common_utils/Utils.hpp"
#include "common_utils/FileSystem.hpp"
#include "ImageCaptureBase.hpp"
#include "sensors/SensorBase.hpp"

namespace msr { namespace airlib {

struct AirSimSettings {
private:
    typedef common_utils::Utils Utils;
    typedef ImageCaptureBase::ImageType ImageType;

public: //types
    static constexpr int kSubwindowCount = 3; //must be >= 3 for now

    static constexpr char const * kVehicleInertialFrame = "VehicleInertialFrame";
    static constexpr char const * kSensorLocalFrame = "SensorLocalFrame";

    struct SubwindowSetting {
        int window_index;
        ImageType image_type;
        bool visible;
        std::string camera_name;

        SubwindowSetting(int window_index_val = 0, ImageType image_type_val = ImageType::Scene, bool visible_val = false, const std::string& camera_name_val = "")
            : window_index(window_index_val), image_type(image_type_val), visible(visible_val), camera_name(camera_name_val)
        {
        }
    };

    struct PawnPath {
        std::string pawn_bp;
        std::string slippery_mat;
        std::string non_slippery_mat;

        PawnPath(const std::string& pawn_bp_val = "",
            const std::string& slippery_mat_val = "/AirSim/VehicleAdv/WheelData/Slippery.Slippery",
            const std::string& non_slippery_mat_val = "/AirSim/VehicleAdv/WheelData/NonSlippery.NonSlippery")
            : pawn_bp(pawn_bp_val), slippery_mat(slippery_mat_val), non_slippery_mat(non_slippery_mat_val)
        {
        }
    };

    struct RCSettings {
        int remote_control_id = -1;
        bool allow_api_when_disconnected = false;
    };

    struct Rotation {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;

        Rotation()
        {
        }

        Rotation(float yaw_val, float pitch_val, float roll_val)
            : yaw(yaw_val), pitch(pitch_val), roll(roll_val)
        {
        }

        static Rotation nanRotation()
        {
            static const Rotation val(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
            return val;
        }
    };


    struct GimbalSetting {
        float stabilization = 0;
        //bool is_world_frame = false;
        Rotation rotation = Rotation::nanRotation();
    };

    struct CaptureSetting {
        //below settings_json are obtained by using Unreal console command (press ~):
        // ShowFlag.VisualizeHDR 1.
        //to replicate camera settings_json to SceneCapture2D
        //TODO: should we use UAirBlueprintLib::GetDisplayGamma()?
        typedef msr::airlib::Utils Utils;
        static constexpr float kSceneTargetGamma = 1.4f;

        int image_type = 0;

        unsigned int width = 256, height = 144; //960 X 540
        float fov_degrees = Utils::nan<float>(); //90.0f
        int auto_exposure_method = -1;   //histogram
        float auto_exposure_speed = Utils::nan<float>(); // 100.0f;
        float auto_exposure_bias = Utils::nan<float>(); // 0;
        float auto_exposure_max_brightness = Utils::nan<float>(); // 0.64f;
        float auto_exposure_min_brightness = Utils::nan<float>(); // 0.03f;
        float auto_exposure_low_percent = Utils::nan<float>(); // 80.0f;
        float auto_exposure_high_percent = Utils::nan<float>(); // 98.3f;
        float auto_exposure_histogram_log_min = Utils::nan<float>(); // -8;
        float auto_exposure_histogram_log_max = Utils::nan<float>(); // 4;
        float motion_blur_amount = Utils::nan<float>();
        float target_gamma = Utils::nan<float>(); //1.0f; //This would be reset to kSceneTargetGamma for scene as default
        int projection_mode = 0; // ECameraProjectionMode::Perspective
        float ortho_width = Utils::nan<float>();
    };

    struct NoiseSetting {
        int ImageType = 0;

        bool Enabled = false;

        float RandContrib = 0.2f;
        float RandSpeed = 100000.0f;
        float RandSize = 500.0f;
        float RandDensity = 2.0f;

        float HorzWaveContrib = 0.03f;
        float HorzWaveStrength = 0.08f;
        float HorzWaveVertSize = 1.0f;
        float HorzWaveScreenSize = 1.0f;

        float HorzNoiseLinesContrib = 1.0f;
        float HorzNoiseLinesDensityY = 0.01f;
        float HorzNoiseLinesDensityXY = 0.5f;

        float HorzDistortionContrib = 1.0f;
        float HorzDistortionStrength = 0.002f;
    };

    struct CameraSetting {
        //nan means keep the default values set in components
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();

        GimbalSetting gimbal;
        std::map<int, CaptureSetting> capture_settings;
        std::map<int, NoiseSetting>  noise_settings;

        CameraSetting()
        {
            initializeCaptureSettings(capture_settings);
            initializeNoiseSettings(noise_settings);
        }
    };

    struct CameraDirectorSetting {
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();
        float follow_distance = Utils::nan<float>();
    };

    struct SensorSetting {
        SensorBase::SensorType sensor_type;
        std::string sensor_name;
        bool enabled;
    };

    struct ImuSetting : SensorSetting {
    };

    struct GpsSetting : SensorSetting {
    };

    struct DistanceSetting : SensorSetting {
    };

    struct LidarSetting : SensorSetting {

        // shared defaults
        uint number_of_lasers = 16;
        real_T range = 10000.0f / 100;                    // meters
        uint points_per_scan = 10000;
        uint horizontal_rotation_frequency = 10;          // rotations/sec
        float horizontal_FOV_start = 0;                   // degrees
        float horizontal_FOV_end = 359;                   // degrees

        // defaults specific to a mode
        float vertical_FOV_upper = Utils::nan<float>();   // drones -15, car +10
        float vertical_FOV_lower = Utils::nan<float>();   // drones -45, car -10
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();

        bool draw_debug_points = false;
    };

    struct VehicleSetting {
        //required
        std::string vehicle_name;

        //optional
        std::string default_vehicle_state;
        std::string pawn_path;
        bool allow_api_always = true;
        bool auto_create = true;
        bool enable_collision_passthrough = false;
        bool enable_trace = false;
        bool enable_collisions = true;
        bool is_fpv_vehicle = false;

        //nan means use player start
        Vector3r position = VectorMath::nanVector(); //in global NED
        Rotation rotation = Rotation::nanRotation();

        std::map<std::string, CameraSetting> cameras;
        std::map<std::string, std::unique_ptr<SensorSetting>> sensors;

        RCSettings rc;
    };

    struct MavLinkConnectionInfo {
        /* Default values are requires so uninitialized instance doesn't have random values */

        bool use_serial = true; // false means use UDP or TCP instead

        //Used to connect via HITL: needed only if use_serial = true
        std::string serial_port = "*";
        int baud_rate = 115200;

        // Used to connect to drone over UDP: needed only if use_serial = false and use_tcp == false
        std::string udp_address = "127.0.0.1";
        int udp_port = 14560;

        // Used to accept connections from drone over TCP: needed only if use_tcp = true
        bool use_tcp = false;
        int tcp_port = 4560;

        // The PX4 SITL app requires receiving drone commands over a different mavlink channel called
        // the "ground control station" channel.
        // So set this to empty string to disable this separate command channel.
        std::string control_ip_address = "127.0.0.1";
        int control_port = 14580;

        // The log viewer can be on a different machine, so you can configure it's ip address and port here.
        int logviewer_ip_port = 14388;
        int logviewer_ip_sport = 14389; // for logging all messages we send to the vehicle.
        std::string logviewer_ip_address = "";

        // The QGroundControl app can be on a different machine, and AirSim can act as a proxy to forward
        // the mavlink stream over to that machine if you configure it's ip address and port here.
        int qgc_ip_port = 0;
        std::string qgc_ip_address = "";

        // mavlink vehicle identifiers
        uint8_t sim_sysid = 142;
        int sim_compid = 42;
        uint8_t offboard_sysid = 134;
        int offboard_compid = 1;
        uint8_t vehicle_sysid = 135;
        int vehicle_compid = 1;

        // if you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet)
        // then you will want to change the LocalHostIp accordingly.  This default only works when log viewer and QGC are also on the
        // same machine.  Whatever network you choose it has to be the same one for external
        std::string local_host_ip = "127.0.0.1";

        std::string model = "Generic";

        std::map<std::string, float> params;
    };

	struct MavLinkVehicleSetting : public VehicleSetting {
		MavLinkConnectionInfo connection_info;
	};

    struct SegmentationSetting {
        enum class InitMethodType {
            None, CommonObjectsRandomIDs
        };

        enum class MeshNamingMethodType {
            OwnerName, StaticMeshName
        };

        InitMethodType init_method = InitMethodType::CommonObjectsRandomIDs;
        bool override_existing = false;
        MeshNamingMethodType mesh_naming_method = MeshNamingMethodType::OwnerName;
    };

    struct TimeOfDaySetting {
        bool enabled = false;
        std::string start_datetime = "";    //format: %Y-%m-%d %H:%M:%S
        bool is_start_datetime_dst = false;
        float celestial_clock_speed = 1;
        float update_interval_secs = 60;
        bool move_sun = true;
    };

private: //fields
    float settings_version_actual;
    float settings_version_minimum = 1.2f;

public: //fields
    std::vector<SubwindowSetting> subwindow_settings;
    SegmentationSetting segmentation_setting;
    TimeOfDaySetting tod_setting;

    std::vector<std::string> warning_messages;
    std::vector<std::string> error_messages;

    int initial_view_mode = 2; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME
    bool enable_rpc = true;
    std::string api_server_address = "";
	int api_port = RpcLibPort;

    // If the current game is running as a listen server, and a spectator connects,
    // it must proide this password to be allowed to enter.
    std::string spectator_server_password = "password";

    std::string clock_type = "";
    float clock_speed = 1.0f;
    bool log_messages_visible = true;
    HomeGeoPoint origin_geopoint{ GeoPoint(47.641468, -122.140165, 122) }; //The geo-coordinate assigned to Unreal coordinate 0,0,0
    std::map<std::string, PawnPath> pawn_paths; //path for pawn blueprint
    std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    CameraSetting camera_defaults;
    CameraDirectorSetting camera_director;
	float speed_unit_factor =  1.0f;
	std::string speed_unit_label = "m\\s";
    std::map<std::string, std::unique_ptr<SensorSetting>> sensor_defaults;

public: //methods
    static AirSimSettings& singleton()
    {
        static AirSimSettings instance;
        return instance;
    }

    AirSimSettings()
    {
        initializeSubwindowSettings(subwindow_settings);
        initializePawnPaths(pawn_paths);
        initializeVehicleSettings(vehicles);
    }

    //returns number of warnings
    void load()
    {
        warning_messages.clear();
        error_messages.clear();
        const Settings& settings_json = Settings::singleton();
        checkSettingsVersion(settings_json);

        loadDefaultCameraSetting(settings_json, camera_defaults);
        loadCameraDirectorSetting(settings_json, camera_director);
        loadSubWindowsSettings(settings_json, subwindow_settings);
        loadViewModeSettings(settings_json);
        loadSegmentationSetting(settings_json, segmentation_setting);
        loadPawnPaths(settings_json, pawn_paths);
        loadOtherSettings(settings_json);
        loadDefaultSensorSettings(settings_json, sensor_defaults);
        loadVehicleSettings(settings_json, vehicles);

        //this should be done last because it depends on type of vehicles we have
        loadClockSettings(settings_json);
    }

    static void initializeSettings(const std::string& json_settings_text)
    {
        try {
            Settings::loadJSonString(json_settings_text);
        }
        catch (std::exception &ex) {
            throw std::invalid_argument(std::string("Error while parsing settings.json: ") + ex.what());
        }
    }

    const VehicleSetting* getVehicleSetting(const std::string& vehicle_name) const
    {
        auto it = vehicles.find(vehicle_name);
        if (it == vehicles.end())
            throw std::invalid_argument(Utils::stringf("VehicleSetting for vehicle name %s was requested but not found",
                vehicle_name.c_str()).c_str());
        else
            return it->second.get();
    }

private:
    void checkSettingsVersion(const Settings& settings_json)
    {
        bool has_default_settings = hasDefaultSettings(settings_json, settings_version_actual);
        bool upgrade_required = settings_version_actual < settings_version_minimum;
        if (upgrade_required) {
            bool auto_upgrade = false;

            //if we have default setting file not modified by user then we will
            //just auto-upgrade it
            if (has_default_settings) {
                auto_upgrade = true;
            }
            else {
                //check if auto-upgrade is possible
                if (settings_version_actual == 1) {
                    const std::vector<std::string> all_changed_keys = {
                        "AdditionalCameras", "CaptureSettings", "NoiseSettings",
                        "UsageScenario", "SimpleFlight", "PX4"
                    };
                    std::stringstream detected_keys_ss;
                    for (const auto& changed_key : all_changed_keys) {
                        if (settings_json.hasKey(changed_key))
                            detected_keys_ss << changed_key << ",";
                    }
                    std::string detected_keys = detected_keys_ss.str();
                    if (detected_keys.length()) {
                        std::string error_message =
                            "You are using newer version of AirSim with older version of settings.json. "
                            "You can either delete your settings.json and restart AirSim or use the upgrade "
                            "instructions at https://git.io/vjefh. \n\n"
                            "Following keys in your settings.json needs updating: "
                            ;

                        error_messages.push_back(error_message + detected_keys);
                    }
                    else
                        auto_upgrade = true;
                }
                else
                    auto_upgrade = true;
            }

            if (auto_upgrade) {
                warning_messages.push_back(
                    "You are using newer version of AirSim with older version of settings.json. "
                    "You should delete your settings.json file and restart AirSim.");
            }
        }
        //else no action necessary
    }

    bool hasDefaultSettings(const Settings& settings_json, float& version)
    {
        //if empty settings file
        bool has_default = settings_json.size() == 0;

        bool has_docs = settings_json.getString("SeeDocsAt", "") != ""
            || settings_json.getString("see_docs_at", "") != "";
        //we had spelling mistake so we are currently supporting SettingsVersion or SettingdVersion :(
        version = settings_json.getFloat("SettingsVersion", settings_json.getFloat("SettingdVersion", 0));

        //If we have pre-V1 settings and only element is docs link
        has_default |= settings_json.size() == 1 && has_docs;

        //if we have V1 settings and only elements are docs link and version
        has_default |= settings_json.size() == 2 && has_docs && version > 0;

        return has_default;
    }

    void loadViewModeSettings(const Settings& settings_json)
    {
        std::string view_mode_string = settings_json.getString("ViewMode", "");

        if (view_mode_string == "") {
            view_mode_string = "SpringArmChase";
        }

        if (view_mode_string == "Fpv")
            initial_view_mode = 0; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV;
        else if (view_mode_string == "GroundObserver")
            initial_view_mode = 1; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER;
        else if (view_mode_string == "FlyWithMe")
            initial_view_mode = 2; //ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME;
        else if (view_mode_string == "Manual")
            initial_view_mode = 3; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL;
        else if (view_mode_string == "SpringArmChase")
            initial_view_mode = 4; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE;
        else if (view_mode_string == "Backup")
            initial_view_mode = 5; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_BACKUP;
        else if (view_mode_string == "NoDisplay")
            initial_view_mode = 6; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_NODISPLAY;
        else if (view_mode_string == "Front")
            initial_view_mode = 7; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_FRONT;
        else
            error_messages.push_back("ViewMode setting is not recognized: " + view_mode_string);
    }

    static void loadRCSetting(const Settings& settings_json, RCSettings& rc_setting)
    {
        Settings rc_json;
        if (settings_json.getChild("RC", rc_json)) {
            rc_setting.remote_control_id = rc_json.getInt("RemoteControlID", -1);
            rc_setting.allow_api_when_disconnected = rc_json.getBool("AllowAPIWhenDisconnected",
                rc_setting.allow_api_when_disconnected);
        }
    }

    static std::string getCameraName(const Settings& settings_json)
    {
        return settings_json.getString("CameraName",
            //TODO: below exist only due to legacy reason and can be replaced by "" in future
            std::to_string(settings_json.getInt("CameraID", 0)));
    }

    static void initializeCaptureSettings(std::map<int, CaptureSetting>& capture_settings)
    {
        capture_settings.clear();
        for (int i = -1; i < Utils::toNumeric(ImageType::Count); ++i) {
            capture_settings[i] = CaptureSetting();
        }
        capture_settings.at(Utils::toNumeric(ImageType::Scene)).target_gamma = CaptureSetting::kSceneTargetGamma;
    }

    static void loadCaptureSettings(const Settings& settings_json, std::map<int, CaptureSetting>& capture_settings)
    {
        initializeCaptureSettings(capture_settings);

        Settings json_parent;
        if (settings_json.getChild("CaptureSettings", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child)) {
                    CaptureSetting capture_setting;
                    createCaptureSettings(json_settings_child, capture_setting);
                    capture_settings[capture_setting.image_type] = capture_setting;
                }
            }
        }
    }

    static std::unique_ptr<VehicleSetting> createMavLinkVehicleSetting(const Settings& settings_json)
    {
        //these settings_json are expected in same section, not in another child
        std::unique_ptr<VehicleSetting> vehicle_setting_p = std::unique_ptr<VehicleSetting>(new MavLinkVehicleSetting());
		MavLinkVehicleSetting* vehicle_setting = static_cast<MavLinkVehicleSetting*>(vehicle_setting_p.get());

        //TODO: we should be selecting remote if available else keyboard
        //currently keyboard is not supported so use rc as default
        vehicle_setting->rc.remote_control_id = 0;

        MavLinkConnectionInfo &connection_info = vehicle_setting->connection_info;
        connection_info.sim_sysid = static_cast<uint8_t>(settings_json.getInt("SimSysID", connection_info.sim_sysid));
        connection_info.sim_compid = settings_json.getInt("SimCompID", connection_info.sim_compid);

        connection_info.vehicle_sysid = static_cast<uint8_t>(settings_json.getInt("VehicleSysID", connection_info.vehicle_sysid));
        connection_info.vehicle_compid = settings_json.getInt("VehicleCompID", connection_info.vehicle_compid);

        connection_info.offboard_sysid = static_cast<uint8_t>(settings_json.getInt("OffboardSysID", connection_info.offboard_sysid));
        connection_info.offboard_compid = settings_json.getInt("OffboardCompID", connection_info.offboard_compid);

        connection_info.logviewer_ip_address = settings_json.getString("LogViewerHostIp", connection_info.logviewer_ip_address);
        connection_info.logviewer_ip_port = settings_json.getInt("LogViewerPort", connection_info.logviewer_ip_port);
        connection_info.logviewer_ip_sport = settings_json.getInt("LogViewerSendPort", connection_info.logviewer_ip_sport);

        connection_info.qgc_ip_address = settings_json.getString("QgcHostIp", connection_info.qgc_ip_address);
        connection_info.qgc_ip_port = settings_json.getInt("QgcPort", connection_info.qgc_ip_port);

        connection_info.control_ip_address = settings_json.getString("ControlIp", connection_info.control_ip_address);
        connection_info.control_port = settings_json.getInt("ControlPort", connection_info.control_port);

        std::string sitlip = settings_json.getString("SitlIp", connection_info.control_ip_address);
        if (sitlip.size() > 0 && connection_info.control_ip_address.size() == 0)
        {
            // backwards compat
            connection_info.control_ip_address = sitlip;
        }
        if (settings_json.hasKey("SitlPort"))
        {
            // backwards compat
            connection_info.control_port = settings_json.getInt("SitlPort", connection_info.control_port);
        }

        connection_info.local_host_ip = settings_json.getString("LocalHostIp", connection_info.local_host_ip);

        connection_info.use_serial = settings_json.getBool("UseSerial", connection_info.use_serial);
        connection_info.udp_address = settings_json.getString("UdpIp", connection_info.udp_address);
        connection_info.udp_port = settings_json.getInt("UdpPort", connection_info.udp_port);
        connection_info.use_tcp = settings_json.getBool("UseTcp", connection_info.use_tcp);
        connection_info.tcp_port = settings_json.getInt("TcpPort", connection_info.tcp_port);
        connection_info.serial_port = settings_json.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = settings_json.getInt("SerialBaudRate", connection_info.baud_rate);
        connection_info.model = settings_json.getString("Model", connection_info.model);

        Settings params;
        if (settings_json.getChild("Parameters", params)) {
            std::vector<std::string> keys;
            params.getChildNames(keys);
            for (auto key: keys)
            {
                connection_info.params[key] = params.getFloat(key, 0);
            }
        }

        return vehicle_setting_p;
    }

    static Vector3r createVectorSetting(const Settings& settings_json, const Vector3r& default_vec)
    {
        return Vector3r(settings_json.getFloat("X", default_vec.x()),
            settings_json.getFloat("Y", default_vec.y()),
            settings_json.getFloat("Z", default_vec.z()));
    }
    static Rotation createRotationSetting(const Settings& settings_json, const Rotation& default_rot)
    {
        return Rotation(settings_json.getFloat("Yaw", default_rot.yaw),
            settings_json.getFloat("Pitch", default_rot.pitch),
            settings_json.getFloat("Roll", default_rot.roll));
    }

    static std::unique_ptr<VehicleSetting> createVehicleSetting(const Settings& settings_json,
        const std::string vehicle_name)
    {

        std::unique_ptr<VehicleSetting> vehicle_setting;

        vehicle_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());        
        vehicle_setting->vehicle_name = vehicle_name;


        //optional settings_json
        vehicle_setting->pawn_path = settings_json.getString("PawnPath", "");
        vehicle_setting->default_vehicle_state = settings_json.getString("DefaultVehicleState", "");
        vehicle_setting->allow_api_always = settings_json.getBool("AllowAPIAlways",
            vehicle_setting->allow_api_always);
        vehicle_setting->auto_create = settings_json.getBool("AutoCreate",
            vehicle_setting->auto_create);
        vehicle_setting->enable_collision_passthrough = settings_json.getBool("EnableCollisionPassthrogh",
            vehicle_setting->enable_collision_passthrough);
        vehicle_setting->enable_trace = settings_json.getBool("EnableTrace",
            vehicle_setting->enable_trace);
        vehicle_setting->enable_collisions = settings_json.getBool("EnableCollisions",
            vehicle_setting->enable_collisions);
        vehicle_setting->is_fpv_vehicle = settings_json.getBool("IsFpvVehicle",
            vehicle_setting->is_fpv_vehicle);

        loadRCSetting(settings_json, vehicle_setting->rc);

        vehicle_setting->position = createVectorSetting(settings_json, vehicle_setting->position);
        vehicle_setting->rotation = createRotationSetting(settings_json, vehicle_setting->rotation);

        loadCameraSettings(settings_json, vehicle_setting->cameras);
        loadSensorSettings(settings_json, "Sensors", vehicle_setting->sensors);

        return vehicle_setting;
    }

    static void initializeVehicleSettings(std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        vehicles.clear();

        //NOTE: Do not set defaults for vehicle type here. If you do then make sure
        //to sync code in createVehicleSetting() as well.

        //create default car vehicle
        auto physx_car_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
        physx_car_setting->vehicle_name = "PhysXCar";
        vehicles[physx_car_setting->vehicle_name] = std::move(physx_car_setting);
    }

    static void loadVehicleSettings(const Settings& settings_json,
        std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        initializeVehicleSettings(vehicles);

        msr::airlib::Settings vehicles_child;
        if (settings_json.getChild("Vehicles", vehicles_child)) {
            std::vector<std::string> keys;
            vehicles_child.getChildNames(keys);

            //remove default vehicles, if values are specified in settings
            if (keys.size())
                vehicles.clear();

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                vehicles_child.getChild(key, child);
                vehicles[key] = createVehicleSetting(child, key);
            }
        }
    }

    static void initializePawnPaths(std::map<std::string, PawnPath>& pawn_paths)
    {
        pawn_paths.clear();
        pawn_paths.emplace("DefaultCar",
            PawnPath("Class'/AirSim/VehicleAdv/Cars/TechnionCar/TechnionCarPawn.TechnionCarPawn_C'"));
    }

    static void loadPawnPaths(const Settings& settings_json, std::map<std::string, PawnPath>& pawn_paths)
    {
        initializePawnPaths(pawn_paths);

        msr::airlib::Settings pawn_paths_child;
        if (settings_json.getChild("PawnPaths", pawn_paths_child)) {
            std::vector<std::string> keys;
            pawn_paths_child.getChildNames(keys);

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                pawn_paths_child.getChild(key, child);
                pawn_paths[key] = createPathPawn(child);
            }
        }
    }

    static PawnPath createPathPawn(const Settings& settings_json)
    {
        auto paths = PawnPath();
        paths.pawn_bp = settings_json.getString("PawnBP", "");
        auto slippery_mat = settings_json.getString("SlipperyMat", "");
        auto non_slippery_mat = settings_json.getString("NonSlipperyMat", "");

        if (slippery_mat != "")
            paths.slippery_mat = slippery_mat;
        if (non_slippery_mat != "")
            paths.non_slippery_mat = non_slippery_mat;

        return paths;
    }

    static void loadSegmentationSetting(const Settings& settings_json, SegmentationSetting& segmentation_setting)
    {
        Settings json_parent;
        if (settings_json.getChild("SegmentationSettings", json_parent)) {
            std::string init_method = Utils::toLower(json_parent.getString("InitMethod", ""));
            if (init_method == "" || init_method == "commonobjectsrandomids")
                segmentation_setting.init_method = SegmentationSetting::InitMethodType::CommonObjectsRandomIDs;
            else if (init_method == "none")
                segmentation_setting.init_method = SegmentationSetting::InitMethodType::None;
            else
                //TODO: below exception doesn't actually get raised right now because of issue in Unreal Engine?
                throw std::invalid_argument(std::string("SegmentationSetting init_method has invalid value in settings_json ") + init_method);

            segmentation_setting.override_existing = json_parent.getBool("OverrideExisting", false);

            std::string mesh_naming_method = Utils::toLower(json_parent.getString("MeshNamingMethod", ""));
            if (mesh_naming_method == "" || mesh_naming_method == "ownername")
                segmentation_setting.mesh_naming_method = SegmentationSetting::MeshNamingMethodType::OwnerName;
            else if (mesh_naming_method == "staticmeshname")
                segmentation_setting.mesh_naming_method = SegmentationSetting::MeshNamingMethodType::StaticMeshName;
            else
                throw std::invalid_argument(std::string("SegmentationSetting MeshNamingMethod has invalid value in settings_json ") + mesh_naming_method);
        }
    }

    static void initializeNoiseSettings(std::map<int, NoiseSetting>&  noise_settings)
    {
        int image_count = Utils::toNumeric(ImageType::Count);
        noise_settings.clear();
        for (int i = -1; i < image_count; ++i)
            noise_settings[i] = NoiseSetting();
    }

    static void loadNoiseSettings(const Settings& settings_json, std::map<int, NoiseSetting>&  noise_settings)
    {
        initializeNoiseSettings(noise_settings);

        Settings json_parent;
        if (settings_json.getChild("NoiseSettings", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child)) {
                    NoiseSetting noise_setting;
                    loadNoiseSetting(json_settings_child, noise_setting);
                    noise_settings[noise_setting.ImageType] = noise_setting;
                }
            }
        }
    }

    static void loadNoiseSetting(const msr::airlib::Settings& settings_json, NoiseSetting& noise_setting)
    {
        noise_setting.Enabled = settings_json.getBool("Enabled", noise_setting.Enabled);
        noise_setting.ImageType = settings_json.getInt("ImageType", noise_setting.ImageType);

        noise_setting.HorzWaveStrength = settings_json.getFloat("HorzWaveStrength", noise_setting.HorzWaveStrength);
        noise_setting.RandSpeed = settings_json.getFloat("RandSpeed", noise_setting.RandSpeed);
        noise_setting.RandSize = settings_json.getFloat("RandSize", noise_setting.RandSize);
        noise_setting.RandDensity = settings_json.getFloat("RandDensity", noise_setting.RandDensity);
        noise_setting.RandContrib = settings_json.getFloat("RandContrib", noise_setting.RandContrib);
        noise_setting.HorzWaveContrib = settings_json.getFloat("HorzWaveContrib", noise_setting.HorzWaveContrib);
        noise_setting.HorzWaveVertSize = settings_json.getFloat("HorzWaveVertSize", noise_setting.HorzWaveVertSize);
        noise_setting.HorzWaveScreenSize = settings_json.getFloat("HorzWaveScreenSize", noise_setting.HorzWaveScreenSize);
        noise_setting.HorzNoiseLinesContrib = settings_json.getFloat("HorzNoiseLinesContrib", noise_setting.HorzNoiseLinesContrib);
        noise_setting.HorzNoiseLinesDensityY = settings_json.getFloat("HorzNoiseLinesDensityY", noise_setting.HorzNoiseLinesDensityY);
        noise_setting.HorzNoiseLinesDensityXY = settings_json.getFloat("HorzNoiseLinesDensityXY", noise_setting.HorzNoiseLinesDensityXY);
        noise_setting.HorzDistortionStrength = settings_json.getFloat("HorzDistortionStrength", noise_setting.HorzDistortionStrength);
        noise_setting.HorzDistortionContrib = settings_json.getFloat("HorzDistortionContrib", noise_setting.HorzDistortionContrib);
    }

    static GimbalSetting createGimbalSetting(const Settings& settings_json)
    {
        GimbalSetting gimbal;
        //capture_setting.gimbal.is_world_frame = settings_json.getBool("IsWorldFrame", false);
        gimbal.stabilization = settings_json.getFloat("Stabilization", false);
        gimbal.rotation = createRotationSetting(settings_json, gimbal.rotation);
        return gimbal;
    }

    static CameraSetting createCameraSetting(const Settings& settings_json)
    {
        CameraSetting setting;

        setting.position = createVectorSetting(settings_json, setting.position);
        setting.rotation = createRotationSetting(settings_json, setting.rotation);

        loadCaptureSettings(settings_json, setting.capture_settings);
        loadNoiseSettings(settings_json, setting.noise_settings);
        Settings json_gimbal;
        if (settings_json.getChild("Gimbal", json_gimbal))
            setting.gimbal = createGimbalSetting(json_gimbal);

        return setting;
    }

    static void loadCameraSettings(const Settings& settings_json, std::map<std::string, CameraSetting>& cameras)
    {
        cameras.clear();

        Settings json_parent;
        if (settings_json.getChild("Cameras", json_parent)) {
            std::vector<std::string> keys;
            json_parent.getChildNames(keys);

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                json_parent.getChild(key, child);
                cameras[key] = createCameraSetting(child);
            }
        }
    }

    static void createCaptureSettings(const msr::airlib::Settings& settings_json, CaptureSetting& capture_setting)
    {
        capture_setting.width = settings_json.getInt("Width", capture_setting.width);
        capture_setting.height = settings_json.getInt("Height", capture_setting.height);
        capture_setting.fov_degrees = settings_json.getFloat("FOV_Degrees", capture_setting.fov_degrees);
        capture_setting.auto_exposure_speed = settings_json.getFloat("AutoExposureSpeed", capture_setting.auto_exposure_speed);
        capture_setting.auto_exposure_bias = settings_json.getFloat("AutoExposureBias", capture_setting.auto_exposure_bias);
        capture_setting.auto_exposure_max_brightness = settings_json.getFloat("AutoExposureMaxBrightness", capture_setting.auto_exposure_max_brightness);
        capture_setting.auto_exposure_min_brightness = settings_json.getFloat("AutoExposureMinBrightness", capture_setting.auto_exposure_min_brightness);
        capture_setting.motion_blur_amount = settings_json.getFloat("MotionBlurAmount", capture_setting.motion_blur_amount);
        capture_setting.image_type = settings_json.getInt("ImageType", 0);
        capture_setting.target_gamma = settings_json.getFloat("TargetGamma",
            capture_setting.image_type == 0 ? CaptureSetting::kSceneTargetGamma : Utils::nan<float>());

        std::string projection_mode = Utils::toLower(settings_json.getString("ProjectionMode", ""));
        if (projection_mode == "" || projection_mode == "perspective")
            capture_setting.projection_mode = 0; // Perspective
        else if (projection_mode == "orthographic")
            capture_setting.projection_mode = 1; // Orthographic
        else
            throw std::invalid_argument(std::string("CaptureSettings projection_mode has invalid value in settings_json ") + projection_mode);

        capture_setting.ortho_width = settings_json.getFloat("OrthoWidth", capture_setting.ortho_width);
    }

    static void loadSubWindowsSettings(const Settings& settings_json, std::vector<SubwindowSetting>& subwindow_settings)
    {
        //load default subwindows
        initializeSubwindowSettings(subwindow_settings);

        Settings json_parent;
        if (settings_json.getChild("SubWindows", json_parent)) {
            for (size_t child_index = 0; child_index < json_parent.size(); ++child_index) {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child)) {
                    int window_index = json_settings_child.getInt("WindowID", 0);
                    SubwindowSetting& subwindow_setting = subwindow_settings.at(window_index);
                    subwindow_setting.window_index = window_index;
                    subwindow_setting.image_type = Utils::toEnum<ImageType>(
                        json_settings_child.getInt("ImageType", 0));
                    subwindow_setting.visible = json_settings_child.getBool("Visible", false);
                    subwindow_setting.camera_name = getCameraName(json_settings_child);
                }
            }
        }
    }

    static void initializeSubwindowSettings(std::vector<SubwindowSetting>& subwindow_settings)
    {
        subwindow_settings.clear();
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::DepthVis, false, "")); //depth
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Segmentation, false, "")); //seg
        subwindow_settings.push_back(SubwindowSetting(0, ImageType::Scene, false, "")); //vis
    }

    void loadOtherSettings(const Settings& settings_json)
    {
        //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
        //because for docker container default is 0.0.0.0 and people get really confused why things
        //don't work
        api_server_address = settings_json.getString("LocalHostIp", "");
		api_port = settings_json.getInt("ApiServerPort", RpcLibPort);
        enable_rpc = settings_json.getBool("EnableRpc", enable_rpc);
        speed_unit_factor = settings_json.getFloat("SpeedUnitFactor", 1.0f);
        speed_unit_label = settings_json.getString("SpeedUnitLabel", "m\\s");
        log_messages_visible = settings_json.getBool("LogMessagesVisible", true);
        spectator_server_password = settings_json.getString("SpectatorServerPassword", "password");

        {   //load origin geopoint
            Settings origin_geopoint_json;
            if (settings_json.getChild("OriginGeopoint", origin_geopoint_json)) {
                GeoPoint origin = origin_geopoint.home_geo_point;
                origin.latitude = origin_geopoint_json.getDouble("Latitude", origin.latitude);
                origin.longitude = origin_geopoint_json.getDouble("Longitude", origin.longitude);
                origin.altitude = origin_geopoint_json.getFloat("Altitude", origin.altitude);
                origin_geopoint.initialize(origin);
            }
        }

        {   //time of day settings_json
            Settings tod_settings_json;
            if (settings_json.getChild("TimeOfDay", tod_settings_json)) {
                tod_setting.enabled = tod_settings_json.getBool("Enabled", tod_setting.enabled);
                tod_setting.start_datetime = tod_settings_json.getString("StartDateTime", tod_setting.start_datetime);
                tod_setting.celestial_clock_speed = tod_settings_json.getFloat("CelestialClockSpeed", tod_setting.celestial_clock_speed);
                tod_setting.is_start_datetime_dst = tod_settings_json.getBool("StartDateTimeDst", tod_setting.is_start_datetime_dst);
                tod_setting.update_interval_secs = tod_settings_json.getFloat("UpdateIntervalSecs", tod_setting.update_interval_secs);
                tod_setting.move_sun = tod_settings_json.getBool("MoveSun", tod_setting.move_sun);
            }
        }
    }

    static void loadDefaultCameraSetting(const Settings& settings_json, CameraSetting& camera_defaults)
    {
        Settings child_json;
        if (settings_json.getChild("CameraDefaults", child_json)) {
            camera_defaults = createCameraSetting(child_json);
        }
    }

    static void loadCameraDirectorSetting(const Settings& settings_json,
        CameraDirectorSetting& camera_director)
    {
        camera_director = CameraDirectorSetting();

        Settings child_json;
        if (settings_json.getChild("CameraDirector", child_json)) {
            camera_director.position = createVectorSetting(settings_json, camera_director.position);
            camera_director.rotation = createRotationSetting(settings_json, camera_director.rotation);
            camera_director.follow_distance = child_json.getFloat("FollowDistance", camera_director.follow_distance);
        }

        if (std::isnan(camera_director.follow_distance)) {
            camera_director.follow_distance = -8;
        }
        if (std::isnan(camera_director.position.x()))
            camera_director.position.x() = camera_director.follow_distance;
        if (std::isnan(camera_director.position.y()))
            camera_director.position.y() = 0;
        if (std::isnan(camera_director.position.z())) {
            camera_director.position.z() = -4;
        }
    }

    void loadClockSettings(const Settings& settings_json)
    {
        clock_type = settings_json.getString("ClockType", "");

        if (clock_type == "") {
            //default value
            clock_type = "ScalableClock";
        }

        clock_speed = settings_json.getFloat("ClockSpeed", 1.0f);
    }

    static void initializeImuSetting(ImuSetting& imu_setting, const Settings& settings_json)
    {
        unused(imu_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeGpsSetting(GpsSetting& gps_setting, const Settings& settings_json)
    {
        unused(gps_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeDistanceSetting(DistanceSetting& distance_setting, const Settings& settings_json)
    {
        unused(distance_setting);
        unused(settings_json);

        //TODO: set from json as needed
    }

    static void initializeLidarSetting(LidarSetting& lidar_setting, const Settings& settings_json)
    {
        lidar_setting.number_of_lasers = settings_json.getInt("NumberOfLasers", lidar_setting.number_of_lasers);
        lidar_setting.range = settings_json.getFloat("Range", lidar_setting.range);
        lidar_setting.points_per_scan = settings_json.getInt("PointsPerScan", lidar_setting.points_per_scan);
        lidar_setting.horizontal_rotation_frequency = settings_json.getInt("RotationsPerSecond", lidar_setting.horizontal_rotation_frequency);
        lidar_setting.draw_debug_points = settings_json.getBool("DrawDebugPoints", lidar_setting.draw_debug_points);

        lidar_setting.vertical_FOV_upper = settings_json.getFloat("VerticalFOVUpper", lidar_setting.vertical_FOV_upper);
        lidar_setting.vertical_FOV_lower = settings_json.getFloat("VerticalFOVLower", lidar_setting.vertical_FOV_lower);
        lidar_setting.horizontal_FOV_start = settings_json.getFloat("HorizontalFOVStart", lidar_setting.horizontal_FOV_start);
        lidar_setting.horizontal_FOV_end = settings_json.getFloat("HorizontalFOVEnd", lidar_setting.horizontal_FOV_end);

        lidar_setting.position = createVectorSetting(settings_json, lidar_setting.position);
        lidar_setting.rotation = createRotationSetting(settings_json, lidar_setting.rotation);
    }

    static std::unique_ptr<SensorSetting> createSensorSetting(
        SensorBase::SensorType sensor_type, const std::string& sensor_name,
        bool enabled)
    {
        std::unique_ptr<SensorSetting> sensor_setting;

        switch (sensor_type) {
        case SensorBase::SensorType::Imu:
            sensor_setting = std::unique_ptr<SensorSetting>(new ImuSetting());
            break;
        case SensorBase::SensorType::Gps:
            sensor_setting = std::unique_ptr<SensorSetting>(new GpsSetting());
            break;
        case SensorBase::SensorType::Distance:
            sensor_setting = std::unique_ptr<SensorSetting>(new DistanceSetting());
            break;
        case SensorBase::SensorType::Lidar:
            sensor_setting = std::unique_ptr<SensorSetting>(new LidarSetting());
            break;
        case SensorBase::SensorType::GSS:
            // at this moment the codebase requires every sensor to have settings.
            // However, gss does not have settings... so we just place ImuSettings in here for now.
            sensor_setting = std::unique_ptr<SensorSetting>(new ImuSetting());
            break;
        default:
            throw std::invalid_argument("Unexpected sensor type (B)");
        }

        sensor_setting->sensor_type = sensor_type;
        sensor_setting->sensor_name = sensor_name;
        sensor_setting->enabled = enabled;

        return sensor_setting;
    }

    static void initializeSensorSetting(SensorSetting* sensor_setting, const Settings& settings_json)
    {
        sensor_setting->enabled = settings_json.getBool("Enabled", sensor_setting->enabled);

        switch (sensor_setting->sensor_type) {
        case SensorBase::SensorType::Imu:
            initializeImuSetting(*static_cast<ImuSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Gps:
            initializeGpsSetting(*static_cast<GpsSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Distance:
            initializeDistanceSetting(*static_cast<DistanceSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::Lidar:
            initializeLidarSetting(*static_cast<LidarSetting*>(sensor_setting), settings_json);
            break;
        case SensorBase::SensorType::GSS:
            break;
        default:
            throw std::invalid_argument("Unexpected sensor type (A)");
        }
    }

    // creates and intializes sensor settings from json
    static void loadSensorSettings( const Settings& settings_json, const std::string& collectionName,
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        msr::airlib::Settings sensors_child;
        if (settings_json.getChild(collectionName, sensors_child)) {
            std::vector<std::string> keys;
            sensors_child.getChildNames(keys);

            for (const auto& key : keys) {
                msr::airlib::Settings child;
                sensors_child.getChild(key, child);

                auto sensor_type = Utils::toEnum<SensorBase::SensorType>(child.getInt("SensorType", 0));
                auto enabled = child.getBool("Enabled", false);

                sensors[key] = createSensorSetting(sensor_type, key, enabled);
                initializeSensorSetting(sensors[key].get(), child);
            }
        }
    }

    // creates default sensor list when none specified in json
    static void createDefaultSensorSettings(
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        sensors["gps"] = createSensorSetting(SensorBase::SensorType::Gps, "gps", true);
    }

    // loads or creates default sensor list
    static void loadDefaultSensorSettings(
        const Settings& settings_json,
        std::map<std::string, std::unique_ptr<SensorSetting>>& sensors)
    {
        msr::airlib::Settings sensors_child;
        if (settings_json.getChild("DefaultSensors", sensors_child))
            loadSensorSettings(settings_json, "DefaultSensors", sensors);
        else
            createDefaultSensorSettings(sensors);
    }
};

}} //namespace
#endif
