// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClientBase_hpp
#define air_RpcLibClientBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "physics/Kinematics.hpp"
#include "api/WorldSimApiBase.hpp"

namespace msr { namespace airlib {

//common methods for RCP clients of different vehicles
class RpcLibClientBase {
public:
    enum class ConnectionState : uint {
        Initial = 0, Connected, Disconnected, Reset, Unknown
    };
public:
    RpcLibClientBase(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);
    virtual ~RpcLibClientBase();    //required for pimpl

    void confirmConnection();
    void reset();

    ConnectionState getConnectionState();
    bool ping();
    int getClientVersion() const;
    int getServerVersion() const;
    int getMinRequiredServerVersion() const;
    int getMinRequiredClientVersion() const;

    bool simIsPaused() const;
    void simPause(bool is_paused);
    void simContinueForTime(double seconds);

    void simSetTimeOfDay(bool is_enabled, const string& start_datetime = "", bool is_start_datetime_dst = false,
        float celestial_clock_speed = 1, float update_interval_secs = 60, bool move_sun = true);

    void simEnableWeather(bool enable);
    void simSetWeatherParameter(WorldSimApiBase::WeatherParameter param, float val);

    vector<string> simListSceneObjects(const string& name_regex = string(".*")) const;
    Pose simGetObjectPose(const std::string& object_name) const;
    bool simSetObjectPose(const std::string& object_name, const Pose& pose, bool teleport = true);

    //task management APIs
    void cancelLastTask(const std::string& vehicle_name = "");
    virtual RpcLibClientBase* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>());

    bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false);
    int simGetSegmentationObjectID(const std::string& mesh_name) const;
    void simPrintLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0);

    void simFlushPersistentMarkers();
    void simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent);
    void simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent);
    void simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent);
    void simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent);
    void simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration); 
    void simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent);
    void simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration);

    bool isApiControlEnabled(const std::string& vehicle_name = "") const;
    void enableApiControl(bool is_enabled, const std::string& vehicle_name = "");

    msr::airlib::GeoPoint getHomeGeoPoint(const std::string& vehicle_name = "") const;

    // sensor APIs
    msr::airlib::LidarData getLidarData(const std::string& lidar_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::ImuBase::Output getImuData(const std::string& imu_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::GpsBase::Output getGpsData(const std::string& gps_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::DistanceBase::Output getDistanceSensorData(const std::string& distance_sensor_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::GSSSimple::Output getGroundSpeedSensorData(const std::string& vehicle_name = "") const;

    // sensor omniscient APIs
    vector<int> simGetLidarSegmentation(const std::string& lidar_name = "", const std::string& vehicle_name = "") const;

    Pose simGetVehiclePose(const std::string& vehicle_name = "") const;
    void simSetVehiclePose(const Pose& pose, bool ignore_collision, const std::string& vehicle_name = "");
    void simSetTraceLine(const std::vector<float>& color_rgba, float thickness=3.0f, const std::string& vehicle_name = "");

    vector<ImageCaptureBase::ImageResponse> simGetImages(vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name = "");
    vector<uint8_t> simGetImage(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name = "");

    vector<MeshPositionVertexBuffersResponse> simGetMeshPositionVertexBuffers();

    CollisionInfo simGetCollisionInfo(const std::string& vehicle_name = "") const;

    CameraInfo simGetCameraInfo(const std::string& camera_name, const std::string& vehicle_name = "") const;
    void simSetCameraOrientation(const std::string& camera_name, const Quaternionr& orientation, const std::string& vehicle_name = "");
    void simSetCameraFov(const std::string& camera_name, float fov_degrees, const std::string& vehicle_name = "");

    msr::airlib::Kinematics::State simGetGroundTruthKinematics(const std::string& vehicle_name = "") const;

	std::vector<std::string> simSwapTextures(const std::string& tags, int tex_id = 0, int component_id = 0, int material_id = 0);

protected:
    void* getClient();
    const void* getClient() const;

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif
