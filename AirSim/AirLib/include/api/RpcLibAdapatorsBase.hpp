// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibAdapatorsBase_hpp
#define air_RpcLibAdapatorsBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "common/ImageCaptureBase.hpp"
#include "api/WorldSimApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class RpcLibAdapatorsBase {
public:
    template<typename TSrc, typename TDest>
    static void to(const std::vector<TSrc>& s, std::vector<TDest>& d)
    {
        d.clear();
        for (size_t i = 0; i < s.size(); ++i)
            d.push_back(s.at(i).to());
    }

    template<typename TSrc, typename TDest>
    static void from(const std::vector<TSrc>& s, std::vector<TDest>& d)
    {
        d.clear();
        for (size_t i = 0; i < s.size(); ++i)
            d.push_back(TDest(s.at(i)));
    }

    struct Vector3r {
        msr::airlib::real_T x_val = 0, y_val = 0, z_val = 0;
        MSGPACK_DEFINE_MAP(x_val, y_val, z_val);

        Vector3r()
        {}

        Vector3r(const msr::airlib::Vector3r& s)
        {
            x_val = s.x();
            y_val = s.y();
            z_val = s.z();
        }
        msr::airlib::Vector3r to() const
        {
            return msr::airlib::Vector3r(x_val, y_val, z_val);
        }
    };

    struct CollisionInfo {
        bool has_collided = false;
        Vector3r normal;
        Vector3r impact_point;
        Vector3r position;
        msr::airlib::real_T penetration_depth = 0;
        msr::airlib::TTimePoint time_stamp = 0;
        std::string object_name;
        int object_id = -1;

        MSGPACK_DEFINE_MAP(has_collided, penetration_depth, time_stamp, normal, impact_point, position, object_name, object_id);
        
        CollisionInfo()
        {}

        CollisionInfo(const msr::airlib::CollisionInfo& s)
        {
            has_collided = s.has_collided;
            normal = s.normal;
            impact_point = s.impact_point;
            position = s.position;
            penetration_depth = s.penetration_depth;
            time_stamp = s.time_stamp;
            object_name = s.object_name;
            object_id = s.object_id;
        }

        msr::airlib::CollisionInfo to() const
        {
            return msr::airlib::CollisionInfo(has_collided, normal.to(), impact_point.to(), position.to(),
                penetration_depth, time_stamp, object_name, object_id);
        }
    };

    struct Quaternionr {
        msr::airlib::real_T w_val = 1, x_val = 0, y_val = 0, z_val = 0;
        MSGPACK_DEFINE_MAP(w_val, x_val, y_val, z_val);

        Quaternionr()
        {}

        Quaternionr(const msr::airlib::Quaternionr& s)
        {
            w_val = s.w();
            x_val = s.x();
            y_val = s.y();
            z_val = s.z();
        }
        msr::airlib::Quaternionr to() const
        {
            return msr::airlib::Quaternionr(w_val, x_val, y_val, z_val);
        }
    };

    struct Pose {
        Vector3r position;
        Quaternionr orientation;
        MSGPACK_DEFINE_MAP(position, orientation);

        Pose()
        {}
        Pose(const msr::airlib::Pose& s)
        {
            position = s.position;
            orientation = s.orientation;
        }
        msr::airlib::Pose to() const
        {
            return msr::airlib::Pose(position.to(), orientation.to());
        }
    };

    struct GeoPoint {
        double latitude = 0, longitude = 0;
        float altitude = 0;
        MSGPACK_DEFINE_MAP(latitude, longitude, altitude);

        GeoPoint()
        {}

        GeoPoint(const msr::airlib::GeoPoint& s)
        {
            latitude = s.latitude;
            longitude = s.longitude;
            altitude = s.altitude;
        }
        msr::airlib::GeoPoint to() const
        {
            return msr::airlib::GeoPoint(latitude, longitude, altitude);
        }
    };

    struct RCData {
        uint64_t timestamp = 0;
        float pitch = 0, roll = 0, throttle = 0, yaw = 0;
        float left_z = 0, right_z = 0;
        uint16_t switches = 0;
        std::string vendor_id = "";
        bool is_initialized = false; //is RC connected?
        bool is_valid = false; //must be true for data to be valid

        MSGPACK_DEFINE_MAP(timestamp, pitch, roll, throttle, yaw, left_z, right_z, switches, vendor_id, is_initialized, is_valid);

        RCData()
        {}

        RCData(const msr::airlib::RCData& s)
        {
            timestamp = s.timestamp;
            pitch = s.pitch;
            roll = s.roll;
            throttle = s.throttle;
            yaw = s.yaw;
            left_z = s.left_z;
            right_z = s.right_z;
            switches = s.switches;
            vendor_id = s.vendor_id;
            is_initialized = s.is_initialized;
            is_valid = s.is_valid;

        }
        msr::airlib::RCData to() const
        {
            msr::airlib::RCData d;
            d.timestamp = timestamp;
            d.pitch = pitch;
            d.roll = roll;
            d.throttle = throttle;
            d.yaw = yaw;
            d.left_z = left_z;
            d.right_z = right_z;
            d.switches = switches;
            d.vendor_id = vendor_id;
            d.is_initialized = is_initialized;
            d.is_valid = is_valid;
            
            return d;
        }
    };

    struct ProjectionMatrix {
        float matrix[4][4];

        MSGPACK_DEFINE_MAP(matrix);

        ProjectionMatrix()
        {
        }

        ProjectionMatrix(const msr::airlib::ProjectionMatrix& s)
        {
            for (auto i = 0; i < 4; ++i)
                for (auto j = 0; j < 4; ++j)
                    matrix[i][j] = s.matrix[i][j];
        }

        msr::airlib::ProjectionMatrix to() const
        {
            msr::airlib::ProjectionMatrix s;
            for (auto i = 0; i < 4; ++i)
                for (auto j = 0; j < 4; ++j)
                    s.matrix[i][j] = matrix[i][j];
            return s;
        }
    };

    struct CameraInfo {
        Pose pose;
        float fov;
        ProjectionMatrix proj_mat;

        MSGPACK_DEFINE_MAP(pose, fov, proj_mat);

        CameraInfo()
        {}

        CameraInfo(const msr::airlib::CameraInfo& s)
        {
            pose = s.pose;
            fov = s.fov;
            proj_mat = ProjectionMatrix(s.proj_mat);
        }

        msr::airlib::CameraInfo to() const
        {
            msr::airlib::CameraInfo s;
            s.pose = pose.to();
            s.fov = fov;
            s.proj_mat = proj_mat.to();

            return s;
        }
    };
    
    struct KinematicsState {
        Vector3r position;
        Quaternionr orientation;

        Vector3r linear_velocity;
        Vector3r angular_velocity;

        Vector3r linear_acceleration;
        Vector3r angular_acceleration;

        MSGPACK_DEFINE_MAP(position, orientation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration);


        KinematicsState()
        {}

        KinematicsState(const msr::airlib::Kinematics::State& s)
        {
            position = s.pose.position;
            orientation = s.pose.orientation;
            linear_velocity = s.twist.linear;
            angular_velocity = s.twist.angular;
            linear_acceleration = s.accelerations.linear;
            angular_acceleration = s.accelerations.angular;
        }

        msr::airlib::Kinematics::State to() const
        {
            msr::airlib::Kinematics::State s;
            s.pose.position = position.to();
            s.pose.orientation = orientation.to();
            s.twist.linear = linear_velocity.to();
            s.twist.angular = angular_velocity.to();
            s.accelerations.linear = linear_acceleration.to();
            s.accelerations.angular = angular_acceleration.to();

            return s;
        }
    };

    struct ImageRequest {
        std::string camera_name;
        msr::airlib::ImageCaptureBase::ImageType image_type;
        bool pixels_as_float;
        bool compress;

        MSGPACK_DEFINE_MAP(camera_name, image_type, pixels_as_float, compress);

        ImageRequest()
        {}

        ImageRequest(const msr::airlib::ImageCaptureBase::ImageRequest& s)
        {
            camera_name = s.camera_name;
            image_type = s.image_type;
            pixels_as_float = s.pixels_as_float;
            compress = s.compress;
        }

        msr::airlib::ImageCaptureBase::ImageRequest to() const
        {
            msr::airlib::ImageCaptureBase::ImageRequest d;
            d.camera_name = camera_name;
            d.image_type = image_type;
            d.pixels_as_float = pixels_as_float;
            d.compress = compress;

            return d;
        }

        static std::vector<ImageRequest> from(
            const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& request
        ) {
            std::vector<ImageRequest> request_adaptor;
            for (const auto& item : request)
                request_adaptor.push_back(ImageRequest(item));

            return request_adaptor;
        }
        static std::vector<msr::airlib::ImageCaptureBase::ImageRequest> to(
            const std::vector<ImageRequest>& request_adapter
        ) {
            std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request;
            for (const auto& item : request_adapter)
                request.push_back(item.to());

            return request;
        }         
    };

    struct ImageResponse {
        std::vector<uint8_t> image_data_uint8;
        std::vector<float> image_data_float;

        std::string camera_name;
        Vector3r camera_position;
        Quaternionr camera_orientation;
        msr::airlib::TTimePoint time_stamp;
        std::string message;
        bool pixels_as_float;
        bool compress;
        int width, height;
        msr::airlib::ImageCaptureBase::ImageType image_type;

        MSGPACK_DEFINE_MAP(image_data_uint8, image_data_float, camera_position, camera_name,
            camera_orientation, time_stamp, message, pixels_as_float, compress, width, height, image_type);

        ImageResponse()
        {}

        ImageResponse(const msr::airlib::ImageCaptureBase::ImageResponse& s)
        {
            pixels_as_float = s.pixels_as_float;
            
            image_data_uint8 = s.image_data_uint8;
            image_data_float = s.image_data_float;

            //TODO: remove bug workaround for https://github.com/rpclib/rpclib/issues/152
            if (image_data_uint8.size() == 0)
                image_data_uint8.push_back(0);
            if (image_data_float.size() == 0)
                image_data_float.push_back(0);

            camera_name = s.camera_name;
            camera_position = Vector3r(s.camera_position);
            camera_orientation = Quaternionr(s.camera_orientation);
            time_stamp = s.time_stamp;
            message = s.message;
            compress = s.compress;
            width = s.width;
            height = s.height;
            image_type = s.image_type;
        }

        msr::airlib::ImageCaptureBase::ImageResponse to() const
        {
            msr::airlib::ImageCaptureBase::ImageResponse d;

            d.pixels_as_float = pixels_as_float;

            if (! pixels_as_float)
                d.image_data_uint8 = image_data_uint8;
            else
                d.image_data_float = image_data_float;

            d.camera_name = camera_name;
            d.camera_position = camera_position.to();
            d.camera_orientation = camera_orientation.to();
            d.time_stamp = time_stamp;
            d.message = message;
            d.compress = compress;
            d.width = width;
            d.height = height;
            d.image_type = image_type;

            return d;
        }

        static std::vector<msr::airlib::ImageCaptureBase::ImageResponse> to(
            const std::vector<ImageResponse>& response_adapter
        ) {
            std::vector<msr::airlib::ImageCaptureBase::ImageResponse> response;
            for (const auto& item : response_adapter)
                response.push_back(item.to());

            return response;
        }
        static std::vector<ImageResponse> from(
            const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& response
        ) {
            std::vector<ImageResponse> response_adapter;
            for (const auto& item : response)
                response_adapter.push_back(ImageResponse(item));

            return response_adapter;
        }
    };

    struct LidarData {

        msr::airlib::TTimePoint time_stamp;    // timestamp
        std::vector<float> point_cloud;        // data
        Pose pose;

        MSGPACK_DEFINE_MAP(time_stamp, point_cloud, pose);

        LidarData()
        {}

        LidarData(const msr::airlib::LidarData& s)
        {
            time_stamp = s.time_stamp;
            point_cloud = s.point_cloud;

            //TODO: remove bug workaround for https://github.com/rpclib/rpclib/issues/152
            if (point_cloud.size() == 0)
                point_cloud.push_back(0);

            pose = s.pose;
        }

        msr::airlib::LidarData to() const
        {
            msr::airlib::LidarData d;

            d.time_stamp = time_stamp;
            d.point_cloud = point_cloud;
            d.pose = pose.to();

            return d;
        }
    };

    struct ImuData {
        msr::airlib::TTimePoint time_stamp;
        Quaternionr orientation;
        Vector3r angular_velocity;
        Vector3r linear_acceleration;
        msr::airlib::real_T sigma_arw, sigma_vrw;

        MSGPACK_DEFINE_MAP(time_stamp, orientation, angular_velocity, linear_acceleration, sigma_arw, sigma_vrw);

        ImuData()
        {}

        ImuData(const msr::airlib::ImuBase::Output& s)
        {
            time_stamp = s.time_stamp;
            orientation = s.orientation;
            angular_velocity = s.angular_velocity;
            linear_acceleration = s.linear_acceleration;
            sigma_arw = s.sigma_arw;
            sigma_vrw = s.sigma_vrw;
        }

        msr::airlib::ImuBase::Output to() const
        {
            msr::airlib::ImuBase::Output d;

            d.time_stamp = time_stamp;
            d.orientation = orientation.to();
            d.angular_velocity = angular_velocity.to();
            d.linear_acceleration = linear_acceleration.to();
            d.sigma_vrw = sigma_vrw;
            d.sigma_arw = sigma_arw;

            return d;
        }
    };

    struct GnssReport {
        GeoPoint geo_point;
        msr::airlib::real_T eph = 0.0, epv = 0.0; 
        Vector3r velocity;
        uint64_t time_utc = 0;

        MSGPACK_DEFINE_MAP(geo_point, eph, epv, velocity, time_utc);

        GnssReport()
        {}

        GnssReport(const msr::airlib::GpsBase::GnssReport& s)
        {
            geo_point = s.geo_point;
            eph = s.eph;
            epv = s.epv;
            velocity = s.velocity;
            time_utc = s.time_utc;
        }

        msr::airlib::GpsBase::GnssReport to() const
        {
            msr::airlib::GpsBase::GnssReport d;

            d.geo_point = geo_point.to();
            d.eph = eph;
            d.epv = epv;
            d.velocity = velocity.to();
            d.time_utc = time_utc;

            return d;
        }
    };

    struct GpsData {
        msr::airlib::TTimePoint time_stamp;
        GnssReport gnss;

        MSGPACK_DEFINE_MAP(time_stamp, gnss);

        GpsData()
        {}

        GpsData(const msr::airlib::GpsBase::Output& s)
        {
            time_stamp = s.time_stamp;
            gnss = s.gnss;
        }

        msr::airlib::GpsBase::Output to() const
        {
            msr::airlib::GpsBase::Output d;

            d.time_stamp = time_stamp;
            d.gnss = gnss.to();

            return d;
        }
    };

    struct DistanceSensorData {
        msr::airlib::TTimePoint time_stamp;
        msr::airlib::real_T distance;    //meters
        msr::airlib::real_T min_distance;//m
        msr::airlib::real_T max_distance;//m
        Pose relative_pose;

        MSGPACK_DEFINE_MAP(time_stamp, distance, min_distance, max_distance, relative_pose);

        DistanceSensorData()
        {}

        DistanceSensorData(const msr::airlib::DistanceBase::Output& s)
        {
            time_stamp = s.time_stamp;
            distance = s.distance;
            min_distance = s.min_distance;
            max_distance = s.max_distance;
            relative_pose = s.relative_pose;
        }

        msr::airlib::DistanceBase::Output to() const
        {
            msr::airlib::DistanceBase::Output d;

            d.time_stamp = time_stamp;
            d.distance = distance;
            d.min_distance = min_distance;
            d.max_distance = max_distance;
            d.relative_pose = relative_pose.to();

            return d;
        }
    };

    struct MeshPositionVertexBuffersResponse {
        Vector3r position;
        Quaternionr orientation;

        std::vector<float> vertices;
        std::vector<uint32_t> indices;
        std::string name;

        MSGPACK_DEFINE_MAP(position, orientation, vertices, indices, name);

        MeshPositionVertexBuffersResponse()
        {}

        MeshPositionVertexBuffersResponse(const msr::airlib::MeshPositionVertexBuffersResponse& s)
        {
            position = Vector3r(s.position);
            orientation = Quaternionr(s.orientation);

            vertices = s.vertices;
            indices = s.indices;

            if (vertices.size() == 0)
                vertices.push_back(0);
            if (indices.size() == 0)
                indices.push_back(0);

            name = s.name;
        }

        msr::airlib::MeshPositionVertexBuffersResponse to() const
        {
            msr::airlib::MeshPositionVertexBuffersResponse d;
            d.position = position.to();
            d.orientation = orientation.to();
            d.vertices = vertices;
            d.indices = indices;
            d.name = name;

            return d;
        }

        static std::vector<msr::airlib::MeshPositionVertexBuffersResponse> to(
            const std::vector<MeshPositionVertexBuffersResponse>& response_adapter) {
            std::vector<msr::airlib::MeshPositionVertexBuffersResponse> response;
            for (const auto& item : response_adapter)
                response.push_back(item.to());

            return response;
        }

        static std::vector<MeshPositionVertexBuffersResponse> from(
            const std::vector<msr::airlib::MeshPositionVertexBuffersResponse>& response) {
            std::vector<MeshPositionVertexBuffersResponse> response_adapter;
            for (const auto& item : response)
                response_adapter.push_back(MeshPositionVertexBuffersResponse(item));

            return response_adapter;
        }
    };

    struct GSSData {

        msr::airlib::TTimePoint time_stamp;
        Vector3r linear_velocity;

        MSGPACK_DEFINE_MAP(time_stamp, linear_velocity);

        GSSData()
        {}

        GSSData(const msr::airlib::GSSSimple::Output& o)
        {
            time_stamp = o.time_stamp;
            linear_velocity = o.linear_velocity;
        }

        msr::airlib::GSSSimple::Output to() const
        {
            msr::airlib::GSSSimple::Output s;
            s.linear_velocity = linear_velocity.to();
            s.time_stamp = time_stamp;
 
            return s;
        }
    };


};

}} //namespace

MSGPACK_ADD_ENUM(msr::airlib::ImageCaptureBase::ImageType);
MSGPACK_ADD_ENUM(msr::airlib::WorldSimApiBase::WeatherParameter);

#endif
