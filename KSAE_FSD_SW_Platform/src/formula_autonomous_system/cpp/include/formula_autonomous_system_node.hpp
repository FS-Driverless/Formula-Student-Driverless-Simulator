/**
 * @file formula_autonomous_system.hpp
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2025-07-21
 * 
 * @copyright Copyright (c) 2025
 */

#ifndef FORMULA_AUTONOMOUS_SYSTEM_NODE_HPP
#define FORMULA_AUTONOMOUS_SYSTEM_NODE_HPP

// C++
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// FS msgs
#include <fs_msgs/ControlCommand.h>
#include <fs_msgs/FinishedSignal.h>
#include <fs_msgs/GoSignal.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Perception
#include <formula_autonomous_system.hpp>

class FormulaAutonomousSystemNode
{
public:
    FormulaAutonomousSystemNode();
    ~FormulaAutonomousSystemNode();

// Functions
private:
    bool init();

    bool getParameters();

    private: // Main thread
    void mainLoop(){
        double rate = main_loop_rate_; // Hz
        ros::Rate loop_rate(rate);

        while (ros::ok())
        {
            run();
            publish();
            loop_rate.sleep();
        }

        ROS_INFO("FormulaAutonomousSystem: Main thread terminated");
        return;
    }

public: // Function components
    void run();
    void publish();
    void excute(){
        // Thread start
        main_thread_ = std::thread(&FormulaAutonomousSystemNode::mainLoop, this);

        return;
    }

    bool checkEssentialMessages();

    public: // ROS
    // Callback functions
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(lidar_msg_mutex_);
        lidar_msg_ = *msg;
        is_lidar_msg_init_ = true;
    }
    void camera1Callback(const sensor_msgs::Image::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(camera1_msg_mutex_);
        camera1_msg_ = *msg;
        is_camera1_msg_init_ = true;
    }
    void camera2Callback(const sensor_msgs::Image::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(camera2_msg_mutex_);
        camera2_msg_ = *msg;
        is_camera2_msg_init_ = true;
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(imu_msg_mutex_);
        imu_msg_ = *msg;
        is_imu_msg_init_ = true;
    }
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(gps_msg_mutex_);
        gps_msg_ = *msg;
        is_gps_msg_init_ = true;
    }
    void goSignalCallback(const fs_msgs::GoSignal::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(go_signal_msg_mutex_);
        go_signal_msg_ = *msg;
        is_go_signal_msg_init_ = true;
    }

    void publishControl();
    void publishAutonomousMode();
    void publishVehicleOdom();
    void publishNonGroundPointCloud();
    void publishGroundPointCloud();
    void publishDetectedConesMarker();
    void publishProjectedConesImage();
    void publishCenterLineMarker();
    

// Variables
private:
    bool is_initialized_;
    double main_loop_rate_;
    
public: // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers
    ros::Subscriber lidar_sub_;
    ros::Subscriber camera1_sub_;
    ros::Subscriber camera2_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber go_signal_sub_;

    // Input messages
    sensor_msgs::PointCloud2 lidar_msg_;
    sensor_msgs::Image camera1_msg_;
    sensor_msgs::Image camera2_msg_;
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::NavSatFix gps_msg_;
    fs_msgs::GoSignal go_signal_msg_;

    // Message flags
    bool is_lidar_msg_init_;
    bool is_camera1_msg_init_;
    bool is_camera2_msg_init_;
    bool is_imu_msg_init_;
    bool is_gps_msg_init_;
    bool is_go_signal_msg_init_;

    // Mutexes
    std::mutex lidar_msg_mutex_;
    std::mutex camera1_msg_mutex_;
    std::mutex camera2_msg_mutex_;
    std::mutex imu_msg_mutex_;
    std::mutex gps_msg_mutex_;
    std::mutex go_signal_msg_mutex_;

    // Publishers
    ros::Publisher control_pub_;
    ros::Publisher autonomous_mode_pub_;
    ros::Publisher vehicle_odom_pub_;
    ros::Publisher non_ground_point_cloud_pub_;
    ros::Publisher ground_point_cloud_pub_;
    ros::Publisher detected_cones_marker_pub_;
    ros::Publisher projected_cones_image_pub_;
    ros::Publisher center_line_marker_pub_;
    ros::Publisher lap_count_marker_pub_;

    // Output messages
    fs_msgs::ControlCommand control_command_msg_;
    std_msgs::String autonomous_mode_msg_;

    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Threads
    std::thread main_thread_;

    // FormulaAutonomousSystem
    std::unique_ptr<FormulaAutonomousSystem> formula_autonomous_system_;
};


#endif // FORMULA_AUTONOMOUS_SYSTEM_NODE_HPP
