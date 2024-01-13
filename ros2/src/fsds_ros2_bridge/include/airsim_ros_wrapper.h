#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
    STRICT_MODE_ON

#include "statistics.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <rclcpp/rclcpp.hpp>
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <fs_msgs/msg/control_command.hpp>
#include <fs_msgs/srv/reset.hpp>
#include <fs_msgs/msg/go_signal.hpp>
#include <fs_msgs/msg/finished_signal.hpp>
#include <fs_msgs/msg/track.hpp>
#include <fs_msgs/msg/extra_info.hpp>
#include <fs_msgs/msg/wheel_states.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <iostream>
#include <cmath>
#include <math_common.h>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
// #include <ros/callback_queue.h>
// #include <ros/console.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float32.hpp>
#include <unordered_map>
#include <fstream>
#include <curl/curl.h>
#include <image_transport/transport_hints.hpp>
// #include "nodelet/nodelet.h"
#define printVariableNameAndValue(x) std::cout << "The name of variable **" << (#x) << "** and the value of variable is => " << x << "\n"

    // todo move airlib typedefs to separate header file?
typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;
typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;
typedef msr::airlib::CarApiBase CarApiBase;

struct enabledSensors {
    bool lidar = false;
    bool camera = false;
    bool gps = false;
    bool gss = false;
    bool imu = false;

    void print() const {
        std::stringstream ss;
        std::cout << "Printing enabled sensors:" << std::endl;

        if(lidar){
            ss << "Lidar enabled\n";
        }

        if(camera){
            ss << "Camera enabled\n";
        }

        if(gps){
            ss << "GPS enabled\n";
        }

        if(gss){
            ss << "GSS enabled\n";
        }

        if(imu){
            ss << "IMU enabled\n";
        }

        std::cout << ss.str();
    }
};

struct SimpleMatrix
{
    int rows;
    int cols;
    double *data;

    SimpleMatrix(int rows, int cols, double *data)
        : rows(rows), cols(cols), data(data)
    {
    }
};

class AirsimROSWrapper
{
public:
    AirsimROSWrapper(const std::shared_ptr<rclcpp::Node>& nh, const std::string& host_ip, double timeout_sec);
    ~AirsimROSWrapper()= default;

    void initialize_airsim(double timeout_sec);
    void initialize_ros();
    void publish_track();
    void initialize_statistics();

    // ros::AsyncSpinner lidar_async_spinner_;
    bool is_used_lidar_timer_cb_queue_{};
    bool is_used_img_timer_cb_queue_{};

    rclcpp::Time first_ros_ts;
    int64_t first_unreal_ts = -1;

private:
    // STATISTICS objects (where possible,
    // the convention of RpcCall+Statistics
    // or Pub/Sub name + _statistics
    // for naming is followed)
    // These instances need to be initialized elsewhere than where they are declared (ie not here)
    ros_bridge::Statistics setCarControlsStatistics;
    ros_bridge::Statistics getGpsDataStatistics;
    ros_bridge::Statistics getCarStateStatistics;
    ros_bridge::Statistics getImuStatistics;
    ros_bridge::Statistics getGSSStatistics;
    std::vector<ros_bridge::Statistics> getLidarDataVecStatistics;
    ros_bridge::Statistics control_cmd_sub_statistics;
    ros_bridge::Statistics global_gps_pub_statistics;
    ros_bridge::Statistics odom_pub_statistics;
    std::vector<ros_bridge::Statistics> cam_pub_vec_statistics;
    std::vector<ros_bridge::Statistics> lidar_pub_vec_statistics;
    ros_bridge::Statistics imu_pub_statistics;
    ros_bridge::Statistics gss_pub_statistics;

    // create std::vector<Statistics*> which I can use to iterate over all these options 
    // and apply common operations such as print, reset
    // std::vector<ros_bridge::Statistics*> statistics_obj_ptr;

    enabledSensors enabled_sensors;

    // Print all statistics
    void PrintStatistics();

    // Reset statistics
    void ResetStatistics();

    /// ROS timer callbacks
    void odom_cb();    // update drone state from airsim_client_ every nth sec
    void gps_timer_cb();
    void imu_timer_cb();
    void gss_timer_cb();
    void wheel_states_timer_cb();
    void statictf_cb();
    void car_control_cb(fs_msgs::msg::ControlCommand::SharedPtr msg);
    void lidar_timer_cb(const std::string& camera_name, int lidar_index);
    void statistics_timer_cb();
    void go_signal_timer_cb();
	void extra_info_cb();
	void clock_timer_cb();

    /// ROS subscriber callbacks
    void finished_signal_cb(fs_msgs::msg::FinishedSignal::SharedPtr msg);

    [[nodiscard]] static rclcpp::Time make_ts(uint64_t unreal_ts) ;
    // void set_zero_vel_cmd();

    /// ROS service callbacks
    bool reset_srv_cb(std::shared_ptr<fs_msgs::srv::Reset::Request> request, std::shared_ptr<fs_msgs::srv::Reset::Response> response);

    // methods which parse setting json ang generate ros pubsubsrv
    void create_ros_pubs_from_settings_json();
    void append_static_camera_tf(const std::string& vehicle_name, const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(const std::string& vehicle_name, const std::string& lidar_name, const LidarSetting& lidar_setting);
    void append_static_vehicle_tf(const std::string& vehicle_name, const VehicleSetting& vehicle_setting);
    static void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) ;
    static void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) ;
    static void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) ;

    /// utils. todo parse into an Airlib<->ROS conversion class
    [[nodiscard]] tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    [[nodiscard]] msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
    [[nodiscard]] msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;

    [[nodiscard]] nav_msgs::msg::Odometry get_odom_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const;
    [[nodiscard]] sensor_msgs::msg::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data);
    [[nodiscard]] static sensor_msgs::msg::PointCloud2 get_lidar_msg_from_airsim(const std::string &lidar_name, const msr::airlib::LidarData& lidar_data) ;
    static bool equalsMessage(const nav_msgs::msg::Odometry& a, const nav_msgs::msg::Odometry& b);


    static std::string readTextFromFile(const std::string& settingsFilepath);

private:
    std::shared_ptr<rclcpp::Service<fs_msgs::srv::Reset>> reset_srvr_;

    std::string vehicle_name;
    std::string map_frame_id_;
    std::string vehicle_frame_id_;
    CarApiBase::Point2D car_start_pos{}; // In Unreal coordinates


    std::vector<std::string> lidar_names_vec_;
    std::vector<geometry_msgs::msg::TransformStamped> static_tf_msg_vec_;
    std::string mission_name_; // rosparam obtained from launch file
    std::string track_name_; // rosparam obtained from launch file
    bool competition_mode_{};
    rclcpp::Time go_timestamp_;

    std::string host_ip_;
    msr::airlib::CarRpcLibClient airsim_client_;
    msr::airlib::CarRpcLibClient airsim_client_lidar_;

    nav_msgs::msg::Odometry message_enu_previous_;

    std::shared_ptr<rclcpp::Node> nh_;

    // todo not sure if async spinners shuold be inside this class, or should be instantiated in fsds_ros2_bridge.cpp, and cb queues should be public
    // todo for multiple drones with multiple sensors, this won't scale. make it a part of MultiRotorROS?
    // ros::CallbackQueue lidar_timer_cb_queue_;

    // todo race condition
    std::recursive_mutex car_control_mutex_;
    // std::recursive_mutex lidar_mutex_;

    /// ROS tf
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;

    /// ROS Timers.
    rclcpp::TimerBase::SharedPtr odom_update_timer_;
    rclcpp::TimerBase::SharedPtr gps_update_timer_;
    rclcpp::TimerBase::SharedPtr imu_update_timer_;
    rclcpp::TimerBase::SharedPtr gss_update_timer_;
    rclcpp::TimerBase::SharedPtr wheel_states_update_timer_;
    std::vector<rclcpp::TimerBase::SharedPtr> airsim_lidar_update_timers_;
    rclcpp::TimerBase::SharedPtr statistics_timer_;
    rclcpp::TimerBase::SharedPtr go_signal_timer_;
    rclcpp::TimerBase::SharedPtr statictf_timer_;
	rclcpp::TimerBase::SharedPtr extra_info_timer_;
	rclcpp::TimerBase::SharedPtr clock_timer_;

    std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>> lidar_pub_vec_;


    /// ROS publishers
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> global_gps_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> gss_pub;
    std::shared_ptr<rclcpp::Publisher<fs_msgs::msg::WheelStates>> wheel_states_pub;
    std::shared_ptr<rclcpp::Publisher<fs_msgs::msg::Track>> track_pub;
    std::shared_ptr<rclcpp::Publisher<fs_msgs::msg::GoSignal>> go_signal_pub_;
	std::shared_ptr<rclcpp::Publisher<fs_msgs::msg::ExtraInfo>> extra_info_pub;
	std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> clock_pub;

    /// ROS subscribers
    rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_cmd_sub;
    rclcpp::Subscription<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_sub_;
};
