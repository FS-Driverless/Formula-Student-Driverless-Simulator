#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(AirsimROSWrapper, nodelet::Nodelet)
#include "common/AirSimSettings.hpp"

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapper::image_type_int_to_string_map_ = {
    {0, "Scene"},
    {1, "DepthPlanner"},
    {2, "DepthPerspective"},
    {3, "DepthVis"},
    {4, "DisparityNormalized"},
    {5, "Segmentation"},
    {6, "SurfaceNormals"},
    {7, "Infrared"}};

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string& host_ip) : nh_(nh),
                                                                                                                               nh_private_(nh_private),
                                                                                                                               img_async_spinner_(1, &img_timer_cb_queue_),     // a thread for image callbacks to be 'spun' by img_async_spinner_
                                                                                                                               lidar_async_spinner_(1, &lidar_timer_cb_queue_), // same as above, but for lidar
                                                                                                                               airsim_client_(host_ip),
                                                                                                                               airsim_client_images_(host_ip),
                                                                                                                               airsim_client_lidar_(host_ip)
{
    is_used_lidar_timer_cb_queue_ = false;
    is_used_img_timer_cb_queue_ = false;

    initialize_statistics();
    initialize_ros();

    std::cout << "AirsimROSWrapper Initialized!\n";
}

void AirsimROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try
    {
        airsim_client_.confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();

        airsim_client_.enableApiControl(true, vehicle_name);
        airsim_client_.armDisarm(true, vehicle_name); 
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
}

void AirsimROSWrapper::initialize_statistics()
{
    setCarControlsStatistics = ros_bridge::Statistics("setCarControls");
    getGpsDataStatistics = ros_bridge::Statistics("getGpsData");
    getCarStateStatistics = ros_bridge::Statistics("getCarState");
    getImuStatistics = ros_bridge::Statistics("getImu");
    control_cmd_sub_statistics = ros_bridge::Statistics("control_cmd_sub");
    global_gps_pub_statistics = ros_bridge::Statistics("global_gps_pub");
    odom_pub_statistics = ros_bridge::Statistics("odom_pub");
    imu_pub_statistics = ros_bridge::Statistics("imu_pub");

    // Populate statistics obj vector
    // statistics_obj_ptr = {&setCarControlsStatistics, &getGpsDataStatistics, &getCarStateStatistics, &control_cmd_sub_statistics, &global_gps_pub_statistics, &odom_local_ned_pub_statistics};
}

void AirsimROSWrapper::initialize_ros()
{

    // ros params
    double update_odom_every_n_sec;
    double update_gps_every_n_sec;
    double update_imu_every_n_sec;
    double publish_static_tf_every_n_sec;
    nh_private_.getParam("is_vulkan", is_vulkan_);
    nh_private_.getParam("mission_name", mission_name_);
    nh_private_.getParam("track_name", track_name_);
    nh_private_.getParam("update_odom_every_n_sec", update_odom_every_n_sec);
    nh_private_.getParam("update_gps_every_n_sec", update_gps_every_n_sec);
    nh_private_.getParam("update_imu_every_n_sec", update_imu_every_n_sec);
    nh_private_.getParam("publish_static_tf_every_n_sec", publish_static_tf_every_n_sec);

    create_ros_pubs_from_settings_json();
    odom_update_timer_ = nh_private_.createTimer(ros::Duration(update_odom_every_n_sec), &AirsimROSWrapper::odom_cb, this);
    gps_update_timer_ = nh_private_.createTimer(ros::Duration(update_gps_every_n_sec), &AirsimROSWrapper::gps_timer_cb, this);
    imu_update_timer_ = nh_private_.createTimer(ros::Duration(update_imu_every_n_sec), &AirsimROSWrapper::imu_timer_cb, this);
    statictf_timer_ = nh_private_.createTimer(ros::Duration(publish_static_tf_every_n_sec), &AirsimROSWrapper::statictf_cb, this);

    statistics_timer_ = nh_private_.createTimer(ros::Duration(1), &AirsimROSWrapper::statistics_timer_cb, this);
    go_signal_timer_ = nh_private_.createTimer(ros::Duration(1), &AirsimROSWrapper::go_signal_timer_cb, this);
}

// XmlRpc::XmlRpcValue can't be const in this case
void AirsimROSWrapper::create_ros_pubs_from_settings_json()
{
    go_signal_pub_ = nh_private_.advertise<fsds_ros_bridge::GoSignal>("signal/go", 1);
    finished_signal_sub_ = nh_private_.subscribe("/fsds_ros_bridge/signal/finished", 1, &AirsimROSWrapper::finished_signal_cb, this);

    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    static_tf_msg_vec_.clear();
    lidar_pub_vec_.clear();
    // vehicle_imu_map_;
    // callback_queues_.clear();

    image_transport::ImageTransport image_transporter(nh_private_);

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : msr::airlib::AirSimSettings::singleton().vehicles)
    {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        if(curr_vehicle_name != "FSCar") {
            throw std::invalid_argument("Only the FSCar vehicle_name is allowed.");
        }

        set_nans_to_zeros_in_pose(*vehicle_setting);

        vehicle_name = curr_vehicle_name;
        odom_pub = nh_private_.advertise<nav_msgs::Odometry>(curr_vehicle_name + "/odom", 10);
        global_gps_pub = nh_private_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/global_gps", 10);
        imu_pub = nh_private_.advertise<sensor_msgs::Imu>(curr_vehicle_name + "/imu", 10);
        control_cmd_sub = nh_private_.subscribe<fsds_ros_bridge::ControlCommand>(curr_vehicle_name + "/control_command", 1, boost::bind(&AirsimROSWrapper::car_control_cb, this, _1, vehicle_name));

        // iterate over camera map std::map<std::string, CameraSetting> cameras;
        for (auto& curr_camera_elem : vehicle_setting->cameras)
        {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;
            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(curr_vehicle_name, curr_camera_name, camera_setting);
            // camera_setting.gimbal
            std::vector<ImageRequest> current_image_request_vec;
            current_image_request_vec.clear();

            // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
            for (const auto& curr_capture_elem : camera_setting.capture_settings)
            {
                auto& capture_setting = curr_capture_elem.second;

                // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
                // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
                if (!(std::isnan(capture_setting.fov_degrees)))
                {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (capture_setting.image_type == 0 || capture_setting.image_type == 5 || capture_setting.image_type == 6 || capture_setting.image_type == 7)
                    {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanner, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else
                    {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    image_pub_vec_.push_back(image_transporter.advertise(curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type), 1));
                    cam_info_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::CameraInfo>(curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type) + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                    // Fill statistics vector here as well using curr_camera_name
                    ros_bridge::Statistics camera_pub_statistics(curr_camera_name + "_Publisher");
                    ros_bridge::Statistics camera_rpc_statistics(curr_camera_name + "_RpcCaller");
                    cam_pub_vec_statistics.push_back(camera_pub_statistics);
                    simGetImagesVecStatistics.push_back(camera_rpc_statistics);
                    // statistics_obj_ptr.push_back(&camera_pub_statistics);
                    // statistics_obj_ptr.insert(statistics_obj_ptr.end(), {&camera_pub_statistics,& camera_rpc_statistics});
                }
            }
            // push back pair (vector of image captures, current vehicle name)
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));
        }

        // iterate over sensors std::map<std::string, std::unique_ptr<SensorSetting>> sensors;
        for (auto& curr_sensor_map : vehicle_setting->sensors)
        {
            auto& sensor_name = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            switch (sensor_setting->sensor_type)
            {
            case msr::airlib::SensorBase::SensorType::Barometer:
            {
                std::cout << "Barometer" << std::endl;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Imu:
            {
                std::cout << "Imu" << std::endl;                
                break;
            }
            case msr::airlib::SensorBase::SensorType::Gps:
            {
                std::cout << "Gps" << std::endl;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Magnetometer:
            {
                std::cout << "Magnetometer" << std::endl;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Distance:
            {
                std::cout << "Distance" << std::endl;
                break;
            }
            case msr::airlib::SensorBase::SensorType::Lidar:
            {
                std::cout << "Lidar" << std::endl;
                auto lidar_setting = *static_cast<LidarSetting *>(sensor_setting.get());
                set_nans_to_zeros_in_pose(*vehicle_setting, lidar_setting);
                append_static_lidar_tf(curr_vehicle_name, sensor_name, lidar_setting); // todo is there a more readable way to down-cast?
                vehicle_lidar_map_[curr_vehicle_name] = sensor_name;                   // non scalable
                lidar_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::PointCloud2>("/lidar/" + sensor_name, 10));
                lidar_pub_vec_statistics.push_back(ros_bridge::Statistics(sensor_name + "_Publisher"));
                getLidarDataVecStatistics.push_back(ros_bridge::Statistics(sensor_name + "_RpcCaller"));
                // statistics_obj_ptr.insert(statistics_obj_ptr.end(), {&lidar_pub_vec_statistics.back(), &getLidarDataVecStatistics.back()});
                break;
            }
            default:
            {
                throw std::invalid_argument("Unexpected sensor type");
            }
            }
        }
    }

    // todo add per vehicle reset in AirLib API
    reset_srvr_ = nh_private_.advertiseService("reset", &AirsimROSWrapper::reset_srv_cb, this);

    // todo mimic gazebo's /use_sim_time feature which publishes airsim's clock time..via an rpc call?!
    // clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 10);

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (airsim_img_request_vehicle_name_pair_vec_.size() > 0)
    {
        double update_airsim_img_response_every_n_sec;
        nh_private_.getParam("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        bool separate_spinner = true; // todo debugging race condition
        if (separate_spinner)
        {
            ros::TimerOptions timer_options(ros::Duration(update_airsim_img_response_every_n_sec), boost::bind(&AirsimROSWrapper::img_response_timer_cb, this, _1), &img_timer_cb_queue_);
            airsim_img_response_timer_ = nh_private_.createTimer(timer_options);
            is_used_img_timer_cb_queue_ = true;
        }
        else
        {
            airsim_img_response_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_img_response_every_n_sec), &AirsimROSWrapper::img_response_timer_cb, this);
        }
    }

    if (lidar_pub_vec_.size() > 0)
    {
        double update_lidar_every_n_sec;
        nh_private_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
        // nh_private_.setCallbackQueue(&lidar_timer_cb_queue_);
        bool separate_spinner = true; // todo debugging race condition
        if (separate_spinner)
        {
            ros::TimerOptions timer_options(ros::Duration(update_lidar_every_n_sec), boost::bind(&AirsimROSWrapper::lidar_timer_cb, this, _1), &lidar_timer_cb_queue_);
            airsim_lidar_update_timer_ = nh_private_.createTimer(timer_options);
            is_used_lidar_timer_cb_queue_ = true;
        }
        else
        {
            airsim_lidar_update_timer_ = nh_private_.createTimer(ros::Duration(update_lidar_every_n_sec), &AirsimROSWrapper::lidar_timer_cb, this);
        }
    }

    initialize_airsim();
}

ros::Time AirsimROSWrapper::make_ts(uint64_t unreal_ts)
{
    if (first_unreal_ts < 0)
    {
        first_unreal_ts = unreal_ts;
        first_ros_ts = ros::Time::now();
    }

    int64_t relative_unreal_ts = unreal_ts - first_unreal_ts;

    int64_t nsec_part = relative_unreal_ts % 1000000000L;
    int64_t sec_part = relative_unreal_ts / 1000000000L;
    while (nsec_part < 0) {
        nsec_part += 1e9L;
        --sec_part;
    }
    if (sec_part > UINT_MAX)
        throw std::runtime_error("sec_part is out of dual 32-bit range");

    return first_ros_ts + ros::Duration(sec_part, nsec_part);
}

// todo add reset by vehicle_name API to airlib
// todo not async remove waitonlasttask
bool AirsimROSWrapper::reset_srv_cb(fsds_ros_bridge::Reset::Request& request, fsds_ros_bridge::Reset::Response& response)
{
    std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);

    airsim_client_.reset();
    return true; //todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}

nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    nav_msgs::Odometry odom_ned_msg;

    odom_ned_msg.pose.pose.position.x = car_state.getPosition().x();
    odom_ned_msg.pose.pose.position.y = car_state.getPosition().y();
    odom_ned_msg.pose.pose.position.z = car_state.getPosition().z();
    odom_ned_msg.pose.pose.orientation.x = car_state.getOrientation().x();
    odom_ned_msg.pose.pose.orientation.y = car_state.getOrientation().y();
    odom_ned_msg.pose.pose.orientation.z = car_state.getOrientation().z();
    odom_ned_msg.pose.pose.orientation.w = car_state.getOrientation().w();

    odom_ned_msg.twist.twist.linear.x = car_state.kinematics_estimated.twist.linear.x();
    odom_ned_msg.twist.twist.linear.y = car_state.kinematics_estimated.twist.linear.y();
    odom_ned_msg.twist.twist.linear.z = car_state.kinematics_estimated.twist.linear.z();
    odom_ned_msg.twist.twist.angular.x = car_state.kinematics_estimated.twist.angular.x();
    odom_ned_msg.twist.twist.angular.y = car_state.kinematics_estimated.twist.angular.y();
    odom_ned_msg.twist.twist.angular.z = car_state.kinematics_estimated.twist.angular.z();

    return odom_ned_msg;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 AirsimROSWrapper::get_lidar_msg_from_airsim(const std::string &lidar_name, const msr::airlib::LidarData& lidar_data) const
{
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header.frame_id = "fsds/"+lidar_name;

    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x";
        lidar_msg.fields[1].name = "y";
        lidar_msg.fields[2].name = "z";
        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            lidar_msg.fields[d].count = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&data_std[0]);
        std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }
    else
    {
        // msg = []
    }
    return lidar_msg;
}

// todo covariances
sensor_msgs::Imu AirsimROSWrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    imu_msg.header.stamp = make_ts(imu_data.time_stamp);
    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}

fsds_ros_bridge::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    fsds_ros_bridge::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

sensor_msgs::NavSatFix AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

void AirsimROSWrapper::car_control_cb(const fsds_ros_bridge::ControlCommand::ConstPtr& msg, const std::string& vehicle_name)
{
    ros_bridge::ROSMsgCounter counter(&control_cmd_sub_statistics);

    // Only allow positive braking and throttle commands to be passed through
    CarApiBase::CarControls controls;
    controls.throttle = msg->throttle < 0.0 ? 0.0 : msg->throttle;
    controls.steering = msg->steering;
    controls.brake = msg->brake < 0.0 ? 0.0 : msg->brake;
    ros::Time time = msg->header.stamp;

    {
        ros_bridge::Timer timer(&setCarControlsStatistics);
        std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
        airsim_client_.setCarControls(controls, vehicle_name);
        lck.unlock();
    }
}

void AirsimROSWrapper::odom_cb(const ros::TimerEvent& event)
{
    try
    {
        msr::airlib::CarApiBase::CarState state;
        {
            ros_bridge::Timer timer(&getCarStateStatistics);
            std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
            state = airsim_client_.getCarState(vehicle_name);
            lck.unlock();
        }

        nav_msgs::Odometry message = this->get_odom_msg_from_airsim_state(state);
        {
            ros_bridge::ROSMsgCounter counter(&odom_pub_statistics);
            odom_pub.publish(message);
        }
    }
    catch (rpc::rpc_error& e)
    {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API:" << std::endl
                  << msg << std::endl;
    }
}

void AirsimROSWrapper::gps_timer_cb(const ros::TimerEvent& event)
{
 try 
 {
    sensor_msgs::NavSatFix message;
    {
        ros_bridge::Timer timer(&getGpsDataStatistics);
        msr::airlib::GeoPoint gps_location = airsim_client_.getGpsData("Gps", vehicle_name).gnss.geo_point;
        message = get_gps_sensor_msg_from_airsim_geo_point(gps_location);
    }
    message.header.stamp = ros::Time::now();
    {
        ros_bridge::ROSMsgCounter counter(&global_gps_pub_statistics);
        global_gps_pub.publish(message);
    }
 }
 catch (rpc::rpc_error& e)
 {
    std::cout << "error" << std::endl;
    std::string msg = e.get_error().as<std::string>();
    std::cout << "Exception raised by the API:" << std::endl
              << msg << std::endl;
 }
}

void AirsimROSWrapper::imu_timer_cb(const ros::TimerEvent& event)
{
    struct msr::airlib::ImuBase::Output imu_data;
    {
        ros_bridge::Timer timer(&getImuStatistics);
        std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
        auto imu_data = airsim_client_.getImuData("Imu", vehicle_name);
        lck.unlock();
    }

    sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
    imu_msg.header.frame_id = "fsds/" + vehicle_name;
    // imu_msg.header.stamp = ros::Time::now();
    {
        ros_bridge::ROSMsgCounter counter(&imu_pub_statistics);
        imu_pub.publish(imu_msg);
    }
}

void AirsimROSWrapper::statictf_cb(const ros::TimerEvent& event)
{
    for (auto& static_tf_msg : static_tf_msg_vec_)
    {
        static_tf_msg.header.stamp = ros::Time::now();
        static_tf_pub_.sendTransform(static_tf_msg);
    }
}


// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const
{
    if (std::isnan(lidar_setting.position.x()))
        lidar_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(lidar_setting.position.y()))
        lidar_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(lidar_setting.position.z()))
        lidar_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(lidar_setting.rotation.yaw))
        lidar_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(lidar_setting.rotation.pitch))
        lidar_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(lidar_setting.rotation.roll))
        lidar_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::append_static_lidar_tf(const std::string& vehicle_name, const std::string& lidar_name, const LidarSetting& lidar_setting)
{

    geometry_msgs::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = "fsds/" + vehicle_name;
    lidar_tf_msg.child_frame_id = "fsds/" + lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.position.x();
    lidar_tf_msg.transform.translation.y = lidar_setting.position.y();
    lidar_tf_msg.transform.translation.z = lidar_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(lidar_setting.rotation.roll, lidar_setting.rotation.pitch, lidar_setting.rotation.yaw);
    lidar_tf_msg.transform.rotation.x = quat.x();
    lidar_tf_msg.transform.rotation.y = quat.y();
    lidar_tf_msg.transform.rotation.z = quat.z();
    lidar_tf_msg.transform.rotation.w = quat.w();

    static_tf_msg_vec_.push_back(lidar_tf_msg);
}

void AirsimROSWrapper::append_static_camera_tf(const std::string& vehicle_name, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = "fsds/" + vehicle_name;
    static_cam_tf_body_msg.child_frame_id = "fsds/" + camera_name;
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    static_tf_msg_vec_.push_back(static_cam_tf_body_msg);
}

void AirsimROSWrapper::img_response_timer_cb(const ros::TimerEvent& event)
{
    try
    {
        int image_response_idx = 0;
        int ctr = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_)
        {
            std::vector<ImageResponse> img_response;
            {
                ros_bridge::Timer timer(&simGetImagesVecStatistics[ctr]);
                std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
                img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);
                lck.unlock();
            }

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size())
            {
                process_and_publish_img_response(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
            ++ctr;
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl
                  << msg << std::endl;
    }
}

void AirsimROSWrapper::lidar_timer_cb(const ros::TimerEvent& event)
{
    try
    {
        // std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);
        if (lidar_pub_vec_.size() > 0)
        {
            // std::lock_guard<std::recursive_mutex> guard(lidar_mutex_);
            int ctr = 0;
            for (const auto& vehicle_lidar_pair : vehicle_lidar_map_)
            {
                sensor_msgs::PointCloud2 lidar_msg;
                struct msr::airlib::LidarData lidar_data;
                {
                    ros_bridge::Timer timer(&getLidarDataVecStatistics[ctr]);
                    std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
                    lidar_data = airsim_client_lidar_.getLidarData(vehicle_lidar_pair.second, vehicle_lidar_pair.first); // airsim api is imu_name, vehicle_name
                    lck.unlock();
                }
                lidar_msg = get_lidar_msg_from_airsim(vehicle_lidar_pair.second, lidar_data);     // todo make const ptr msg to avoid copy
                lidar_msg.header.frame_id = "fsds/" + vehicle_lidar_pair.second; // sensor frame name. todo add to doc
                lidar_msg.header.stamp = ros::Time::now();

                {
                    ros_bridge::ROSMsgCounter counter(&lidar_pub_vec_statistics[ctr]);
                    lidar_pub_vec_[ctr].publish(lidar_msg);
                }
                ctr++;
            }
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl
                  << msg << std::endl;
    }
}

cv::Mat AirsimROSWrapper::manual_decode_depth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                  const ros::Time curr_ros_time,
                                                                  const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = make_ts(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                        const ros::Time curr_ros_time,
                                                                        const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = make_ts(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt car body frame?
sensor_msgs::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& camera_name,
                                                            const CameraSetting& camera_setting,
                                                            const CaptureSetting& capture_setting) const
{
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = "fsds/" + camera_name;
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = {f_x, 0.0, capture_setting.width / 2.0,
                      0.0, f_x, capture_setting.height / 2.0,
                      0.0, 0.0, 1.0};
    cam_info_msg.P = {f_x, 0.0, capture_setting.width / 2.0, 0.0,
                      0.0, f_x, capture_setting.height / 2.0, 0.0,
                      0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    ros::Time curr_ros_time = ros::Time::now();
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec)
    {
        // if a render request failed for whatever reason, this img will be empty.
        // Attempting to use a make_ts(0) results in ros::Duration runtime error.
        if (curr_img_response.time_stamp == 0)
            continue;

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs
        camera_info_msg_vec_[img_response_idx_internal].header.stamp = curr_ros_time;
        {
            ros_bridge::ROSMsgCounter counter(&cam_pub_vec_statistics[img_response_idx_internal]);
            cam_info_pub_vec_[img_response_idx_internal].publish(camera_info_msg_vec_[img_response_idx_internal]);
        }

        // DepthPlanner / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float)
        {
            image_pub_vec_[img_response_idx_internal].publish(get_depth_img_msg_from_response(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              "fsds/" + curr_img_response.camera_name));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else
        {
            image_pub_vec_[img_response_idx_internal].publish(get_img_msg_from_response(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        "fsds/" + curr_img_response.camera_name));
        }
        img_response_idx_internal++;
    }
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m) const
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows * cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info) const
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}

/* 
    STATISTICS
 */

// TODO: it would be nice to avoid code duplication between Print and Reset
// functions with templates or something similar
void AirsimROSWrapper::PrintStatistics()
{
    // This did not work for some reason
    // for (auto statistics_obj : statistics_obj_ptr)
    // {
    //     statistics_obj->Print();
    // }
    setCarControlsStatistics.Print();
    getGpsDataStatistics.Print();
    getCarStateStatistics.Print();
    getImuStatistics.Print();
    control_cmd_sub_statistics.Print();
    global_gps_pub_statistics.Print();
    odom_pub_statistics.Print();
    imu_pub_statistics.Print();
    
    for (auto &simGetImagesStatistics : simGetImagesVecStatistics)
    {
        simGetImagesStatistics.Print();
    }

    for (auto &getLidarDataStatistics : getLidarDataVecStatistics)
    {
        getLidarDataStatistics.Print();
    }

    // Reset camera statistics
    for (auto &cam_info_pub_statistics : cam_pub_vec_statistics)
    {
        cam_info_pub_statistics.Print();
    }

    // Reset lidar statistics
    for (auto &lidar_pub_statistics : lidar_pub_vec_statistics)
    {
        lidar_pub_statistics.Print();
    }
}

void AirsimROSWrapper::ResetStatistics()
{
    // This did not work for some reason
    // for (auto statistics_obj : statistics_obj_ptr)
    // {
    //     statistics_obj->Reset();
    // }
    setCarControlsStatistics.Reset();
    getGpsDataStatistics.Reset();
    getCarStateStatistics.Reset();
    getImuStatistics.Reset();
    control_cmd_sub_statistics.Reset();
    global_gps_pub_statistics.Reset();
    odom_pub_statistics.Reset();
    imu_pub_statistics.Reset();

    for (auto &simGetImagesStatistics : simGetImagesVecStatistics)
    {
        simGetImagesStatistics.Reset();
    }

    for (auto &getLidarDataStatistics : getLidarDataVecStatistics)
    {
        getLidarDataStatistics.Reset();
    }

    // Reset camera statistics
    for (auto &cam_info_pub_statistics : cam_pub_vec_statistics)
    {
        cam_info_pub_statistics.Reset();
    }

    // Reset lidar statistics
    for (auto &lidar_pub_statistics : lidar_pub_vec_statistics)
    {
        lidar_pub_statistics.Reset();
    }
}

// This callback is executed every 1 second
void AirsimROSWrapper::statistics_timer_cb(const ros::TimerEvent &event)
{
    // The lines below are technically correct but for some reason they introduce a lot of noise in the data, which is exactly what they are not supposed to do
    // long double time_elapsed = event.current_real.nsec - event.last_real.nsec; // nanoseconds for accuracy
    // ros_bridge::Statistics::SetTimeElapsed((double)(time_elapsed*std::pow(10, -9))); // convert back to seconds
    PrintStatistics();
    ResetStatistics();
}

// This callback is executed every 1 second
void AirsimROSWrapper::go_signal_timer_cb(const ros::TimerEvent &event)
{
    fsds_ros_bridge::GoSignal go_signal_msg;
    go_signal_msg.mission = mission_name_;
    go_signal_msg.track = track_name_;
    go_signal_pub_.publish(go_signal_msg);
}

void AirsimROSWrapper::finished_signal_cb(fsds_ros_bridge::FinishedSignalConstPtr msg)
{
    // Get access token
    std::string access_token (std::getenv("OPERATOR_ACCESS_TOKEN"));

    // Send JSON HTTP POST request
    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    std::string json_obj = "{\"access_token\" : \"" + access_token + "\" , \"sender\" : \"AS\" }";

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "charsets: utf-8");

    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:5000/mission/stop");
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_obj.c_str());

    res = curl_easy_perform(curl);
    std::cout << res << std::endl;

    curl_easy_cleanup(curl);
    curl_global_cleanup();
}