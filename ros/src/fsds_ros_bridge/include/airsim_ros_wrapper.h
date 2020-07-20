#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
    STRICT_MODE_ON

#include "statistics.h"
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "ros/ros.h"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <fs_msgs/ControlCommand.h>
#include <fsds_ros_bridge/Reset.h>
#include <fs_msgs/GoSignal.h>
#include <fs_msgs/FinishedSignal.h>
#include <fs_msgs/Track.h>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <math_common.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32.h>
#include <unordered_map>
#include <fstream>
#include <curl/curl.h>
// #include "nodelet/nodelet.h"
#define printVariableNameAndValue(x) std::cout << "The name of variable **" << (#x) << "** and the value of variable is => " << x << "\n"

    // todo move airlib typedefs to separate header file?
typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;
typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;
typedef msr::airlib::CarApiBase CarApiBase;

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
    AirsimROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const std::string& host_ip);
    ~AirsimROSWrapper(){};

    void initialize_airsim();
    void initialize_ros();
    void publish_track();
    void initialize_statistics();

    ros::AsyncSpinner lidar_async_spinner_;
    bool is_used_lidar_timer_cb_queue_;
    bool is_used_img_timer_cb_queue_;

    ros::Time first_ros_ts;
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
    std::vector<ros_bridge::Statistics> getLidarDataVecStatistics;
    ros_bridge::Statistics control_cmd_sub_statistics;
    ros_bridge::Statistics global_gps_pub_statistics;
    ros_bridge::Statistics odom_pub_statistics;
    std::vector<ros_bridge::Statistics> cam_pub_vec_statistics;
    std::vector<ros_bridge::Statistics> lidar_pub_vec_statistics;
    ros_bridge::Statistics imu_pub_statistics;
    
    // create std::vector<Statistics*> which I can use to iterate over all these options 
    // and apply common operations such as print, reset
    // std::vector<ros_bridge::Statistics*> statistics_obj_ptr;

    // Print all statistics
    void PrintStatistics();

    // Reset statistics
    void ResetStatistics();

    /// ROS timer callbacks
    void odom_cb(const ros::TimerEvent& event);    // update drone state from airsim_client_ every nth sec
    void gps_timer_cb(const ros::TimerEvent& event);
    void imu_timer_cb(const ros::TimerEvent& event);
    void statictf_cb(const ros::TimerEvent& event);
    void car_control_cb(const fs_msgs::ControlCommand::ConstPtr& msg, const std::string& vehicle_name);
    void lidar_timer_cb(const ros::TimerEvent& event, const std::string& camera_name, const int lidar_index);
    void statistics_timer_cb(const ros::TimerEvent& event);
    void go_signal_timer_cb(const ros::TimerEvent& event);

    /// ROS subscriber callbacks
    void finished_signal_cb(fs_msgs::FinishedSignalConstPtr msg);

    ros::Time make_ts(uint64_t unreal_ts);
    // void set_zero_vel_cmd();

    /// ROS service callbacks
    bool reset_srv_cb(fsds_ros_bridge::Reset::Request& request, fsds_ros_bridge::Reset::Response& response);

    // methods which parse setting json ang generate ros pubsubsrv
    void create_ros_pubs_from_settings_json();
    void append_static_camera_tf(const std::string& vehicle_name, const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(const std::string& vehicle_name, const std::string& lidar_name, const LidarSetting& lidar_setting);
    void append_static_vehicle_tf(const std::string& vehicle_name, const VehicleSetting& vehicle_setting);
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const;

    /// utils. todo parse into an Airlib<->ROS conversion class
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;

    nav_msgs::Odometry get_odom_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const;
    sensor_msgs::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data);
    sensor_msgs::PointCloud2 get_lidar_msg_from_airsim(const std::string &lidar_name, const msr::airlib::LidarData& lidar_data) const;
    static bool equalsMessage(const nav_msgs::Odometry& a, const nav_msgs::Odometry& b);


private:
    ros::ServiceServer reset_srvr_;

    std::string vehicle_name;
    CarApiBase::Point2D car_start_pos; // In Unreal coordinates


    AirSimSettingsParser airsim_settings_parser_;
    std::vector<std::string> lidar_names_vec_;
    std::vector<geometry_msgs::TransformStamped> static_tf_msg_vec_;
    std::string mission_name_; // rosparam obtained from launch file
    std::string track_name_; // rosparam obtained from launch file

    msr::airlib::CarRpcLibClient airsim_client_;
    msr::airlib::CarRpcLibClient airsim_client_lidar_;

    nav_msgs::Odometry message_enu_previous_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // todo not sure if async spinners shuold be inside this class, or should be instantiated in fsds_ros_bridge.cpp, and cb queues should be public
    // todo for multiple drones with multiple sensors, this won't scale. make it a part of MultiRotorROS?
    ros::CallbackQueue lidar_timer_cb_queue_;

    // todo race condition
    std::recursive_mutex car_control_mutex_;
    // std::recursive_mutex lidar_mutex_;

    /// ROS tf
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;

    /// ROS Timers.
    ros::Timer odom_update_timer_;
    ros::Timer gps_update_timer_;
    ros::Timer imu_update_timer_;
    std::vector<ros::Timer> airsim_lidar_update_timers_;
    ros::Timer statistics_timer_;
    ros::Timer go_signal_timer_;
    ros::Timer statictf_timer_;

    std::vector<ros::Publisher> lidar_pub_vec_;


    /// ROS publishers
    ros::Publisher clock_pub_;
    ros::Publisher odom_pub;
    ros::Publisher global_gps_pub;
    ros::Publisher imu_pub;
    ros::Publisher track_pub;
    ros::Publisher go_signal_pub_;
    
    /// ROS subscribers
    ros::Subscriber control_cmd_sub;
    ros::Subscriber finished_signal_sub_;
};