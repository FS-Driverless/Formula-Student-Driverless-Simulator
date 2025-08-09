/**
 * @file formula_autonomous_system.hpp
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @author MinKyu Cho (chomk2000@hanyang.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2025-07-21
 * 
 * @copyright Copyright (c) 2025
 */

#ifndef FORMULA_AUTONOMOUS_SYSTEM_HPP
#define FORMULA_AUTONOMOUS_SYSTEM_HPP

// C++
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <random>
#include <cmath>

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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

// Eigen
#include <Eigen/Dense>

// OpenCV bridge
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// ==================== Constants ====================
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double EARTH_RADIUS = 6378137.0; // WGS84 Earth radius in meters

// ==================== Types ====================
struct Cone {
    pcl::PointXYZ center;
    std::string color;
    float confidence;
    std::vector<pcl::PointXYZ> points;
};

struct ColorConfidence {
    double yellow_confidence;
    double blue_confidence;
    double orange_confidence;
    double unknown_confidence;
    
    ColorConfidence() : yellow_confidence(0.0), blue_confidence(0.0), 
                       orange_confidence(0.0), unknown_confidence(1.0) {}
};

struct VehicleState {
    Eigen::Vector2d position; // 차량의 현재 위치 (x, y)
    double yaw;               // 차량의 현재 헤딩 (rad)
    double speed;             // 차량의 현재 속도 (m/s)

    VehicleState(double x = 0.0, double y = 0.0, double y_rad = 0.0, double spd = 0.0)
        : position(x, y), yaw(y_rad), speed(spd) {}
};

// Autonomous System States according to Formula Student rules
enum class ASState {
    AS_OFF = 0,        // 자율주행 시스템이 비활성화된 상태
    AS_READY = 1,      // 주행 준비가 완료되어 오퍼레이터의 GO 신호를 대기하는 상태
    AS_DRIVING = 2,    // 자율주행으로 트랙을 주행 중인 상태
};

// Event types that can trigger state transitions
enum class ASEvent {
    SYSTEM_INIT,        // 시스템 초기화 완료
    SYSTEM_READY,       // 모든 서브시스템 준비 완료
    GO_SIGNAL          // 오퍼레이터 GO 신호 수신
};

// State transition result
struct StateTransitionResult {
    bool success;
    ASState previous_state;
    ASState current_state;
    std::string message;
    
    StateTransitionResult(bool s, ASState prev, ASState curr, const std::string& msg)
        : success(s), previous_state(prev), current_state(curr), message(msg) {}
};

// ==================== Parameters ====================

// Perception

struct PerceptionParams
{
    // =================== LiDAR Perception Parameters ===================
    // ROI Extraction
    double lidar_roi_x_min_;
    double lidar_roi_x_max_;
    double lidar_roi_y_min_;
    double lidar_roi_y_max_;
    double lidar_roi_z_min_;
    double lidar_roi_z_max_;

    // Ground Removal (RANSAC)
    int lidar_ransac_iterations_;
    double lidar_ransac_distance_threshold_;
    
    // Clustering (DBSCAN)
    double lidar_dbscan_eps_;
    int lidar_dbscan_min_points_;
    
    // Cone Detection
    double lidar_cone_detection_min_height_;
    double lidar_cone_detection_max_height_;
    double lidar_cone_detection_min_radius_;
    double lidar_cone_detection_max_radius_;
    int lidar_cone_detection_min_points_;
    
    // Color Classification (removed - using camera-based classification only)
    
    // =================== Camera Parameters ===================
    
    // Camera Intrinsics
    double camera_fx_;                    // Focal length X
    double camera_fy_;                    // Focal length Y
    double camera_cx_;                    // Principal point X
    double camera_cy_;                    // Principal point Y
    
    // Camera Extrinsics (Camera coordinate system relative to vehicle base)
    std::vector<double> camera_translation_; // [x, y, z] translation from base to camera
    std::vector<double> camera_rotation_;    // [roll, pitch, yaw] rotation from base to camera (radians)
    
    // Image Processing
    bool camera_enable_preprocessing_;
    double camera_gaussian_blur_sigma_;
    int camera_bilateral_filter_d_;
    
    // HSV Color Thresholds for Cone Detection
    int camera_hsv_window_size_;

    int camera_yellow_hue_min_;
    int camera_yellow_hue_max_;
    int camera_yellow_sat_min_;
    int camera_yellow_val_min_;
    
    int camera_blue_hue_min_;
    int camera_blue_hue_max_;
    int camera_blue_sat_min_;
    int camera_blue_val_min_;
    
    int camera_orange_hue_min_;
    int camera_orange_hue_max_;
    int camera_orange_sat_min_;
    int camera_orange_val_min_;
    
    // =================== LiDAR Extrinsics ===================
    
    // LiDAR Extrinsics (LiDAR coordinate system relative to vehicle base)
    std::vector<double> lidar_translation_; // [x, y, z] translation from base to lidar
    std::vector<double> lidar_rotation_;    // [roll, pitch, yaw] rotation from base to lidar (radians)

    // Print parameters for debugging
    void print() const {
        printf("=== Perception Parameters ===\n");
        printf("\n[LiDAR Parameters]\n");
        printf("  ROI Extraction: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n", lidar_roi_x_min_, lidar_roi_x_max_, lidar_roi_y_min_, lidar_roi_y_max_, lidar_roi_z_min_, lidar_roi_z_max_);
        printf("  RANSAC iterations: %d\n", lidar_ransac_iterations_);
        printf("  RANSAC distance threshold: %.3f\n", lidar_ransac_distance_threshold_);
        printf("  DBSCAN eps: %.3f\n", lidar_dbscan_eps_);
        printf("  DBSCAN min points: %d\n", lidar_dbscan_min_points_);
        printf("  Cone height range: [%.3f, %.3f]\n", lidar_cone_detection_min_height_, lidar_cone_detection_max_height_);
        printf("  Cone radius range: [%.3f, %.3f]\n", lidar_cone_detection_min_radius_, lidar_cone_detection_max_radius_);
        printf("  Cone min points: %d\n", lidar_cone_detection_min_points_);
        printf("  Translation: [%.3f, %.3f, %.3f]\n", lidar_translation_[0], lidar_translation_[1], lidar_translation_[2]);
        printf("  Rotation: [%.3f, %.3f, %.3f] deg\n", lidar_rotation_[0], lidar_rotation_[1], lidar_rotation_[2]);
        
        printf("\n[Camera Parameters]\n");
        printf("  Intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\n", camera_fx_, camera_fy_, camera_cx_, camera_cy_);
        printf("  Translation: [%.3f, %.3f, %.3f]\n", camera_translation_[0], camera_translation_[1], camera_translation_[2]);
        printf("  Rotation: [%.3f, %.3f, %.3f] deg\n", camera_rotation_[0], camera_rotation_[1], camera_rotation_[2]);
        printf("  Preprocessing enabled: %s\n", camera_enable_preprocessing_ ? "true" : "false");
        printf("  Gaussian blur sigma: %.3f\n", camera_gaussian_blur_sigma_);
        printf("  Bilateral filter diameter: %d\n", camera_bilateral_filter_d_);
        printf("  HSV window size: %d\n", camera_hsv_window_size_);
        printf("  Yellow HSV range: H[%d,%d] S[%d,255] V[%d,255]\n", 
               camera_yellow_hue_min_, camera_yellow_hue_max_, camera_yellow_sat_min_, camera_yellow_val_min_);
        printf("  Blue HSV range: H[%d,%d] S[%d,255] V[%d,255]\n", 
               camera_blue_hue_min_, camera_blue_hue_max_, camera_blue_sat_min_, camera_blue_val_min_);
        printf("  Orange HSV range: H[%d,%d] S[%d,255] V[%d,255]\n", 
               camera_orange_hue_min_, camera_orange_hue_max_, camera_orange_sat_min_, camera_orange_val_min_);
    }
    
    bool getParameters(ros::NodeHandle& pnh){
        std::cout << "FormulaAutonomousSystem: Parameters file updated" << std::endl;
        // =================== LiDAR Parameters ===================
        // LiDAR ROI Extraction
        if(!pnh.getParam("/perception/lidar_roi_extraction/x_min", lidar_roi_x_min_)){std::cerr<<"Param perception/lidar_roi_extraction/x_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_roi_extraction/x_max", lidar_roi_x_max_)){std::cerr<<"Param perception/lidar_roi_extraction/x_max has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_roi_extraction/y_min", lidar_roi_y_min_)){std::cerr<<"Param perception/lidar_roi_extraction/y_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_roi_extraction/y_max", lidar_roi_y_max_)){std::cerr<<"Param perception/lidar_roi_extraction/y_max has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_roi_extraction/z_min", lidar_roi_z_min_)){std::cerr<<"Param perception/lidar_roi_extraction/z_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_roi_extraction/z_max", lidar_roi_z_max_)){std::cerr<<"Param perception/lidar_roi_extraction/z_max has error" << std::endl; return false;}

        // LiDAR Ground Removal
        if(!pnh.getParam("/perception/lidar_ground_removal/ransac_iterations", lidar_ransac_iterations_)){std::cerr<<"Param perception/lidar_ground_removal/ransac_iterations has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_ground_removal/ransac_distance_threshold", lidar_ransac_distance_threshold_)){std::cerr<<"Param perception/lidar_ground_removal/ransac_distance_threshold has error" << std::endl; return false;}

        // LiDAR Clustering
        if(!pnh.getParam("/perception/lidar_clustering/dbscan_eps", lidar_dbscan_eps_)){std::cerr<<"Param perception/lidar_clustering/dbscan_eps has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_clustering/dbscan_min_points", lidar_dbscan_min_points_)){std::cerr<<"Param perception/lidar_clustering/dbscan_min_points has error" << std::endl; return false;}

        // LiDAR Cone Detection
        if(!pnh.getParam("/perception/lidar_cone_detection/cone_min_radius", lidar_cone_detection_min_radius_)){std::cerr<<"Param perception/lidar_cone_detection/cone_min_radius has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_cone_detection/cone_max_radius", lidar_cone_detection_max_radius_)){std::cerr<<"Param perception/lidar_cone_detection/cone_max_radius has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_cone_detection/cone_min_points", lidar_cone_detection_min_points_)){std::cerr<<"Param perception/lidar_cone_detection/cone_min_points has error" << std::endl; return false;}

        // LiDAR Extrinsics
        if(lidar_translation_.size() != 3) lidar_translation_.resize(3);
        if(lidar_rotation_.size() != 3) lidar_rotation_.resize(3);
        
        if(!pnh.getParam("/perception/lidar_extrinsics/translation_x", lidar_translation_[0])){std::cerr<<"Param perception/lidar_extrinsics/translation_x has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_extrinsics/translation_y", lidar_translation_[1])){std::cerr<<"Param perception/lidar_extrinsics/translation_y has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_extrinsics/translation_z", lidar_translation_[2])){std::cerr<<"Param perception/lidar_extrinsics/translation_z has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_extrinsics/rotation_roll", lidar_rotation_[0])){std::cerr<<"Param perception/lidar_extrinsics/rotation_roll has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_extrinsics/rotation_pitch", lidar_rotation_[1])){std::cerr<<"Param perception/lidar_extrinsics/rotation_pitch has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/lidar_extrinsics/rotation_yaw", lidar_rotation_[2])){std::cerr<<"Param perception/lidar_extrinsics/rotation_yaw has error" << std::endl; return false;}

        // =================== Camera Parameters ===================
        // Camera Intrinsics
        if(!pnh.getParam("/perception/camera_intrinsics/focal_length_x", camera_fx_)){std::cerr<<"Param perception/camera_intrinsics/focal_length_x has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_intrinsics/focal_length_y", camera_fy_)){std::cerr<<"Param perception/camera_intrinsics/focal_length_y has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_intrinsics/principal_point_x", camera_cx_)){std::cerr<<"Param perception/camera_intrinsics/principal_point_x has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_intrinsics/principal_point_y", camera_cy_)){std::cerr<<"Param perception/camera_intrinsics/principal_point_y has error" << std::endl; return false;}

        // Camera Extrinsics
        if(camera_translation_.size() != 3) camera_translation_.resize(3);
        if(camera_rotation_.size() != 3) camera_rotation_.resize(3);
        
        if(!pnh.getParam("/perception/camera_extrinsics/translation_x", camera_translation_[0])){std::cerr<<"Param perception/camera_extrinsics/translation_x has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_extrinsics/translation_y", camera_translation_[1])){std::cerr<<"Param perception/camera_extrinsics/translation_y has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_extrinsics/translation_z", camera_translation_[2])){std::cerr<<"Param perception/camera_extrinsics/translation_z has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_extrinsics/rotation_roll", camera_rotation_[0])){std::cerr<<"Param perception/camera_extrinsics/rotation_roll has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_extrinsics/rotation_pitch", camera_rotation_[1])){std::cerr<<"Param perception/camera_extrinsics/rotation_pitch has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_extrinsics/rotation_yaw", camera_rotation_[2])){std::cerr<<"Param perception/camera_extrinsics/rotation_yaw has error" << std::endl; return false;}

        // Camera Image Processing
        if(!pnh.getParam("/perception/camera_image_processing/enable_preprocessing", camera_enable_preprocessing_)){std::cerr<<"Param perception/camera_image_processing/enable_preprocessing has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_image_processing/gaussian_blur_sigma", camera_gaussian_blur_sigma_)){std::cerr<<"Param perception/camera_image_processing/gaussian_blur_sigma has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_image_processing/bilateral_filter_diameter", camera_bilateral_filter_d_)){std::cerr<<"Param perception/camera_image_processing/bilateral_filter_diameter has error" << std::endl; return false;}

        // Camera HSV Window Size
        if(!pnh.getParam("/perception/camera_hsv_window_size/window_size", camera_hsv_window_size_)){std::cerr<<"Param perception/camera_hsv_window_size/window_size has error" << std::endl; return false;}

        // Camera HSV Yellow
        if(!pnh.getParam("/perception/camera_hsv_yellow/hue_min", camera_yellow_hue_min_)){std::cerr<<"Param perception/camera_hsv_yellow/hue_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_yellow/hue_max", camera_yellow_hue_max_)){std::cerr<<"Param perception/camera_hsv_yellow/hue_max has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_yellow/saturation_min", camera_yellow_sat_min_)){std::cerr<<"Param perception/camera_hsv_yellow/saturation_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_yellow/value_min", camera_yellow_val_min_)){std::cerr<<"Param perception/camera_hsv_yellow/value_min has error" << std::endl; return false;}

        // Camera HSV Blue
        if(!pnh.getParam("/perception/camera_hsv_blue/hue_min", camera_blue_hue_min_)){std::cerr<<"Param perception/camera_hsv_blue/hue_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_blue/hue_max", camera_blue_hue_max_)){std::cerr<<"Param perception/camera_hsv_blue/hue_max has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_blue/saturation_min", camera_blue_sat_min_)){std::cerr<<"Param perception/camera_hsv_blue/saturation_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_blue/value_min", camera_blue_val_min_)){std::cerr<<"Param perception/camera_hsv_blue/value_min has error" << std::endl; return false;}

        // Camera HSV Orange
        if(!pnh.getParam("/perception/camera_hsv_orange/hue_min", camera_orange_hue_min_)){std::cerr<<"Param perception/camera_hsv_orange/hue_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_orange/hue_max", camera_orange_hue_max_)){std::cerr<<"Param perception/camera_hsv_orange/hue_max has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_orange/saturation_min", camera_orange_sat_min_)){std::cerr<<"Param perception/camera_hsv_orange/saturation_min has error" << std::endl; return false;}
        if(!pnh.getParam("/perception/camera_hsv_orange/value_min", camera_orange_val_min_)){std::cerr<<"Param perception/camera_hsv_orange/value_min has error" << std::endl; return false;}

        return true;
    }
};

// ==================== Localization ====================

struct LocalizationParams
{
    // =================== Reference WGS84 Position ===================
    bool use_user_defined_ref_wgs84_position_;
    double ref_wgs84_latitude_;      // Reference latitude (degrees)
    double ref_wgs84_longitude_;     // Reference longitude (degrees)
    double ref_wgs84_altitude_;      // Reference altitude (meters)

    // =================== Velocity Estimation ===================
    double alpha_velocity_;

    // Print parameters for debugging
    void print() const {
        printf("=== Localization Parameters ===\n");
        printf("\n[Reference WGS84 Position]\n");
        printf("  Use user defined: %s\n", use_user_defined_ref_wgs84_position_ ? "true" : "false");
        printf("  Latitude: %.6f°\n", ref_wgs84_latitude_);
        printf("  Longitude: %.6f°\n", ref_wgs84_longitude_);
        printf("  Altitude: %.3fm\n", ref_wgs84_altitude_);

        printf("\n[Velocity Estimation]\n");
        printf("  Alpha velocity: %.6f\n", alpha_velocity_);
    }

    bool getParameters(ros::NodeHandle& pnh){
        std::cout << "FormulaAutonomousSystem: Localization parameters file updated" << std::endl;
        
        // =================== Localization Parameters ===================
        if(!pnh.getParam("/localization/localization/use_user_defined_ref_wgs84_position", use_user_defined_ref_wgs84_position_)){std::cerr<<"Param localization/use_user_defined_ref_wgs84_position has error" << std::endl; return false;}
        if(!pnh.getParam("/localization/localization/ref_wgs84_latitude", ref_wgs84_latitude_)){std::cerr<<"Param localization/ref_wgs84_latitude has error" << std::endl; return false;}
        if(!pnh.getParam("/localization/localization/ref_wgs84_longitude", ref_wgs84_longitude_)){std::cerr<<"Param localization/ref_wgs84_longitude has error" << std::endl; return false;}
        if(!pnh.getParam("/localization/localization/ref_wgs84_altitude", ref_wgs84_altitude_)){std::cerr<<"Param localization/ref_wgs84_altitude has error" << std::endl; return false;}
        if(!pnh.getParam("/localization/localization/alpha_velocity", alpha_velocity_)){std::cerr<<"Param localization/alpha_velocity has error" << std::endl; return false;}

        return true;
    }
};

// ==================== Planning ====================

struct TrajectoryParams {
    // =================== Basic Trajectory Generation ===================
    double lookahead_distance_;     // Lookahead distance (m)
    double waypoint_spacing_;       // Distance between waypoints (m)
    double default_speed_;          // Default trajectory speed (m/s)
    
    // =================== Cone-based Path Planning ===================
    double max_cone_distance_;      // Maximum cone distance to consider (m)
    double lane_offset_;            // Offset from single cone (m)
    
    // =================== Safety Parameters ===================
    double safety_margin_;          // Safety margin from cones (m)
    
    // Print current parameters
    void print() const {
        printf("=== Trajectory Parameters ===\n");
        printf("Lookahead distance: %.3f m\n", lookahead_distance_);
        printf("Waypoint spacing: %.3f m\n", waypoint_spacing_);
        printf("Default speed: %.3f m/s\n", default_speed_);
        printf("Max cone distance: %.3f m\n", max_cone_distance_);
        printf("Lane offset: %.3f m\n", lane_offset_);
        printf("Safety margin: %.3f m\n", safety_margin_);
    }
    
    // Load parameters from ROS NodeHandle
    bool getParameters(ros::NodeHandle& pnh) {
        std::cout << "FormulaAutonomousSystem: Local planning parameters file updated" << std::endl;
        
        // =================== Trajectory Parameters ===================
        if(!pnh.getParam("/local_planning/trajectory/lookahead_distance", lookahead_distance_)){std::cerr<<"Param local_planning/trajectory/lookahead_distance has error" << std::endl; return false;}
        if(!pnh.getParam("/local_planning/trajectory/waypoint_spacing", waypoint_spacing_)){std::cerr<<"Param local_planning/trajectory/waypoint_spacing has error" << std::endl; return false;}
        if(!pnh.getParam("/local_planning/trajectory/default_speed", default_speed_)){std::cerr<<"Param local_planning/trajectory/default_speed has error" << std::endl; return false;}
        if(!pnh.getParam("/local_planning/trajectory/max_cone_distance", max_cone_distance_)){std::cerr<<"Param local_planning/trajectory/max_cone_distance has error" << std::endl; return false;}
        if(!pnh.getParam("/local_planning/trajectory/lane_offset", lane_offset_)){std::cerr<<"Param local_planning/trajectory/lane_offset has error" << std::endl; return false;}
        if(!pnh.getParam("/local_planning/trajectory/safety_margin", safety_margin_)){std::cerr<<"Param local_planning/trajectory/safety_margin has error" << std::endl; return false;}

        return true;
    }
};

// ==================== Control ====================

struct ControlParams {
    // ===================  Controller Selection =================== 
    std::string lateral_controller_type_;
    
    // =================== Lateral Control: Pure Pursuit ===================
    double pp_lookahead_distance_;    // 전방 주시 거리 (m)
    double pp_max_steer_angle_;       // 최대 조향각 (rad)

    // ===================  Stanley Controller Parameters =================== 
    double stanley_k_gain_; // Stanley 제어기 K 게인

    // =================== Longitudinal Control: PID Controller ===================
    double target_speed_;             // 목표 속도 (m/s)
    double pid_kp_;                   // 속도 제어용 PID - Kp
    double pid_ki_;                   // 속도 제어용 PID - Ki
    double pid_kd_;                   // 속도 제어용 PID - Kd
    double max_throttle_;             // 최대 스로틀 값 (0.0 ~ 1.0)
    
    // =================== Vehicle Specification ===================
    double vehicle_length_;           // 차량 축거 (Wheelbase) (m)

    bool getParameters(ros::NodeHandle& pnh) {
        std::cout << "FormulaAutonomousSystem: Control parameters file updated" << std::endl;
        
        // ===================  Controller Selection =================== 
        if(!pnh.getParam("/control/ControllerSelection/lateral_controller_type", lateral_controller_type_)){std::cerr<<"Param control/ControllerSelection/lateral_controller_type has error" << std::endl; return false;}
        
        // =================== Lateral Control: Pure Pursuit ===================
        if(!pnh.getParam("/control/PurePursuit/lookahead_distance", pp_lookahead_distance_)){std::cerr<<"Param control/PurePursuit/lookahead_distance has error" << std::endl; return false;}
        if(!pnh.getParam("/control/PurePursuit/max_steer_angle", pp_max_steer_angle_)){std::cerr<<"Param control/PurePursuit/max_steer_angle has error" << std::endl; return false;}

        // ===================  Stanley Controller Parameters =================== 
        if(!pnh.getParam("/control/Stanley/k_gain", stanley_k_gain_)){std::cerr<<"Param control/Stanley/k_gain has error" << std::endl; return false;}
        
        // =================== Longitudinal Control: PID Controller ===================
        if(!pnh.getParam("/control/SpeedControl/target_speed", target_speed_)){std::cerr<<"Param control/SpeedControl/target_speed has error" << std::endl; return false;}
        if(!pnh.getParam("/control/SpeedControl/pid_kp", pid_kp_)){std::cerr<<"Param control/SpeedControl/pid_kp has error" << std::endl; return false;}
        if(!pnh.getParam("/control/SpeedControl/pid_ki", pid_ki_)){std::cerr<<"Param control/SpeedControl/pid_ki has error" << std::endl; return false;}
        if(!pnh.getParam("/control/SpeedControl/pid_kd", pid_kd_)){std::cerr<<"Param control/SpeedControl/pid_kd has error" << std::endl; return false;}
        if(!pnh.getParam("/control/SpeedControl/max_throttle", max_throttle_)){std::cerr<<"Param control/SpeedControl/max_throttle has error" << std::endl; return false;}

        if(!pnh.getParam("/control/Vehicle/wheel_base", vehicle_length_)){std::cerr<<"Param control/Vehicle/wheel_base has error" << std::endl; return false;}
        
        return true;
    }
};

// ==================== Algorithm ====================

// Perception

class RoiExtractor {
public:
    RoiExtractor(const std::shared_ptr<PerceptionParams> params);
    
    void extractRoi(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& roi_cloud);

private:
    std::shared_ptr<PerceptionParams> params_;
};

class GroundRemoval {
public:
    // Constructor with parameter struct
    explicit GroundRemoval(const std::shared_ptr<PerceptionParams> params);
    
    // Legacy constructor for backward compatibility
    GroundRemoval(double distance_threshold = 0.2, int max_iterations = 1000);
    
    void removeGround(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& non_ground_points
    );

private:
    std::shared_ptr<PerceptionParams> params_;
    
    // RANSAC 3-point plane fitting
    bool fitPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector4f& plane_coefficients,
        std::vector<int>& inliers
    );
    
    double pointToPlaneDistance(const pcl::PointXYZ& point, const Eigen::Vector4f& plane);
    std::mt19937 rng_;
};

class Clustering {
public:
    // Constructor with parameter struct
    explicit Clustering(const std::shared_ptr<PerceptionParams> params);
    
    // Legacy constructor for backward compatibility
    Clustering(double eps = 0.5, int min_points = 10, double min_cone_height = 0.1, double max_cone_height = 0.4);
    
    bool extractCones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_points, std::vector<Cone>& cones);

private:
    std::shared_ptr<PerceptionParams> params_;
    Eigen::Matrix4f vehicle_to_lidar_transform_;
    
    // DBSCAN clustering
    std::vector<std::vector<int>> dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    std::vector<int> regionQuery(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int point_idx,
        const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree
    );
    
    // Cone validation and color classification
    bool isValidCone(const std::vector<int>& cluster_indices, 
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    Eigen::Vector3f calculateCentroid(const std::vector<int>& cluster_indices,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};


class ColorDetection
{
public:
    explicit ColorDetection(const std::shared_ptr<PerceptionParams>& params);
    
    // Main color detection function
    std::string detectConeColor(const Cone& cone, const cv::Mat& rgb_image);
    
    // Batch processing for multiple cones
    std::vector<Cone> classifyConesColor(const std::vector<Cone>& cones, const cv::Mat& rgb_image);
    
    // Utility functions
    cv::Point2f projectToCamera(const pcl::PointXYZ& point_3d);
    bool isPointInImage(const cv::Point2f& point, const cv::Size& image_size);
    
    // Debug visualization
    cv::Mat visualizeProjection(const std::vector<Cone>& cones, const cv::Mat& rgb_image);
    
    // Get computed transformation matrices (for debugging)
    cv::Mat getCameraToBaseRotation() const { return camera_to_base_rotation_.clone(); }
    cv::Mat getCameraToBaseTranslation() const { return camera_to_base_translation_.clone(); }

private:
    std::shared_ptr<PerceptionParams> params_;
    
    // Camera intrinsic matrix
    cv::Mat camera_matrix_;
    
    // Computed transformation from camera to lidar coordinate system
    cv::Mat camera_to_base_rotation_;    // 3x3 rotation matrix
    cv::Mat camera_to_base_translation_; // 3x1 translation vector
    
    // Initialize camera parameters and compute transformations
    void initializeCameraParameters();
    void computeCameraToLidarTransform();
    
    // Helper function to create rotation matrix from euler angles
    cv::Mat createTransformationMatrix(double x, double y, double z, double roll, double pitch, double yaw);
    
    // Color analysis functions
    ColorConfidence analyzeColorWindow(const cv::Mat& hsv_image, const cv::Point2f& center, int window_size);
    
    int countYellowPixels(const cv::Mat& hsv_roi);
    int countBluePixels(const cv::Mat& hsv_roi);
    int countOrangePixels(const cv::Mat& hsv_roi);
    
    std::string selectBestColor(const ColorConfidence& confidence);
    
    // Image preprocessing
    cv::Mat preprocessImage(const cv::Mat& rgb_image);
    
    // Helper functions
    bool isInHSVRange(const cv::Vec3b& hsv_pixel, int hue_min, int hue_max, int sat_min, int val_min);
    cv::Rect getSafeWindow(const cv::Point2f& center, int window_size, const cv::Size& image_size);
};

// Localization

class Localization {
public:
    explicit Localization(const std::shared_ptr<LocalizationParams>& params);
    ~Localization() = default;

public:
    // Update with IMU data (acceleration, yaw rate and IMU orientation)
    void updateImu(const Eigen::Vector3d& imu_input, Eigen::Quaterniond& q_imu, double curr_time_sec);
    
    // Update with GPS data (position in WGS84)
    void updateGps(const Eigen::Vector2d& gps_wgs84, double curr_time_sec);
    
    // Getters
    std::vector<double> getCurrentState() const { return state_; }
    Eigen::Vector3d getCurrentPose() const { return Eigen::Vector3d(state_[0], state_[1], state_[2]); }
    double getCurrentVelocity() const { return state_[3]; }
    double getCurrentYaw() const { return state_[2]; }
    double getCurrentYawRate() const { return state_[5]; }
    double getCurrentAcceleration() const { return state_[6]; }
    double getCurrentLateralAcceleration() const { return state_[7]; }
    
    // Transform utilities
    Eigen::Vector2d wgs84ToEnu(const Eigen::Vector2d& wgs84_pos) const;

private:
    std::vector<double> predictState(const std::vector<double>& state, double dt);

private:
    std::shared_ptr<LocalizationParams> params_;

    std::vector<double> state_; // [x, y, yaw, vx, vy, yawrate, ax, ay]
    double last_time_sec_;

    Eigen::Vector2d ref_wgs84_position_; // [lat, lon]
    Eigen::Vector2d prev_gps_enu_; // [x, y]
    double prev_gps_time_sec_;
};

// Planning
class StateMachine {
public:
    StateMachine();
    ~StateMachine() = default;

    // State management
    ASState getCurrentState() const { return current_state_; }
    std::string getCurrentStateString() const { return stateToString(current_state_); }
    bool isValidTransition(ASState from, ASState to) const;
    
    // Event processing
    StateTransitionResult processEvent(ASEvent event);
    
    // Mission management
    std::string getCurrentMission() const { return current_mission_; }
    
    // Event injection (instead of ROS callbacks)
    void injectSystemInit();
    void injectGoSignal(const std::string& mission, const std::string& track);
    
    // Debug and monitoring
    void printStateInfo() const;
    double getTimeInCurrentState() const;
    
private:
    // State transition handlers
    bool enterAS_OFF();
    bool enterAS_READY();
    bool enterAS_DRIVING();
    
    bool exitAS_OFF();
    bool exitAS_READY();
    bool exitAS_DRIVING();
    
    // Internal state management
    bool performStateTransition(ASState new_state, const std::string& reason);
    
    // Utility functions
    std::string stateToString(ASState state) const;
    std::string eventToString(ASEvent event) const;
    void initializeValidTransitions();
    void logStateTransition(ASState from, ASState to, const std::string& reason);
    
private:
    // State management
    ASState current_state_;
    ASState previous_state_;
    std::chrono::steady_clock::time_point state_entry_time_;
    std::chrono::steady_clock::time_point last_update_time_;
    
    // Mission information
    std::string current_mission_;
    std::string mission_track_;
    bool mission_active_;
    
    // Valid state transitions (finite state machine definition)
    std::map<std::pair<ASState, ASState>, bool> valid_transitions_;
};

// Trajectory waypoint
struct TrajectoryPoint {
    Eigen::Vector2d position;   // x, y position
    double yaw;                 // heading angle (radians)
    double curvature;           // path curvature (1/radius)
    double speed;               // target speed (m/s)
    double s;                   // arc length from start
    
    TrajectoryPoint(double x = 0.0, double y = 0.0, double yaw_val = 0.0, 
                   double curv = 0.0, double spd = 0.0, double s_val = 0.0)
        : position(x, y), yaw(yaw_val), curvature(curv), speed(spd), s(s_val) {}
};

// Trajectory Generator class
class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(const std::shared_ptr<TrajectoryParams>& params);
    ~TrajectoryGenerator() = default;
    
    // Generate trajectory from blue (left) and yellow (right) cones
    std::vector<TrajectoryPoint> generateTrajectory(const std::vector<Cone>& cones, ASState planning_state);
    std::vector<TrajectoryPoint> generateConesTrajectory(const std::vector<Cone>& cones);
    std::vector<TrajectoryPoint> generateStopTrajectory();
    
    /**
     * @brief Update trajectory parameters
     * @param params New parameters
     */
    void updateParams(const std::shared_ptr<TrajectoryParams>& params) { params_ = params; }
    
    /**
     * @brief Get current parameters
     * @return Current parameters
     */
    const std::shared_ptr<TrajectoryParams>& getParams() const { return params_; }
    
    /**
     * @brief Get last generated trajectory
     * @return Last trajectory
     */
    const std::vector<TrajectoryPoint>& getLastTrajectory() const { return last_trajectory_; }
    
    /**
     * @brief Print trajectory statistics
     */
    void printTrajectoryStats() const;

private:
    // Utility functions
    double calculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
    double calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
    
    // Member variables
    std::shared_ptr<TrajectoryParams> params_;
    std::vector<TrajectoryPoint> last_trajectory_;
    
    // Statistics
    int generated_trajectories_;
    double average_generation_time_;
    double last_generation_time_;
};

// Control
// Lateral Controller
class LateralController 
{
public:
    virtual ~LateralController() = default;
    virtual double calculateSteeringAngle(const VehicleState& current_state, const std::vector<TrajectoryPoint>& path) const = 0;
};
// Pure Pursuit Controller
class PurePursuit : public LateralController
{
public:
    explicit PurePursuit(const std::shared_ptr<ControlParams>& params);
    
    double calculateSteeringAngle(const VehicleState& current_state, const std::vector<TrajectoryPoint>& path) const override;

private:
    // 내부 헬퍼 함수들은 그대로 유지
    int findTargetPointIndex(const std::vector<TrajectoryPoint>& path) const;
    double calculateSteeringAngleInternal(const Eigen::Vector2d& target_point) const;

    std::shared_ptr<ControlParams> params_; 
};

// Stanley Controller
class Stanley : public LateralController
{
public:
    explicit Stanley(const std::shared_ptr<ControlParams>& params);

    double calculateSteeringAngle(const VehicleState& current_state, const std::vector<TrajectoryPoint>& path) const override;

private:
    std::shared_ptr<ControlParams> params_;
};

/**
 * @class PIDController
 * @brief 표준 PID(Proportional-Integral-Derivative) 제어기 클래스
 */
class PIDController
{
public:
    /**
     * @brief PID 제어기 생성자
     * @param kp 비례(Proportional) 게인
     * @param ki 적분(Integral) 게인
     * @param kd 미분(Derivative) 게인
     * @param min_output 출력값의 최솟값
     * @param max_output 출력값의 최댓값
     */
    PIDController(const std::shared_ptr<ControlParams>& params);

    /**
     * @brief 제어값을 계산합니다.
     * @param setpoint 목표값 (Desired value)
     * @param measured_value 현재 측정값 (Actual value)
     * @return double 계산된 제어 출력값
     */
    double calculate(double setpoint, double measured_value);

    /**
     * @brief 제어기의 내부 상태(적분항, 이전 오차)를 초기화합니다.
     */
    void reset();

private:
    std::shared_ptr<ControlParams> params_;

    // PID 게인
    double kp_;
    double ki_;
    double kd_;

    // 출력 제한
    double min_output_;
    double max_output_;

    // 제어기 내부 상태 변수
    double integral_error_;
    double previous_error_;
    
    // 시간 변화량(dt) 계산을 위한 변수
    std::chrono::steady_clock::time_point last_time_;
    bool first_run_;
};

// ==================== Formula Autonomous System ====================

class FormulaAutonomousSystem
{
public:
    FormulaAutonomousSystem();
    ~FormulaAutonomousSystem();

// Functions
public:
    bool init(ros::NodeHandle& pnh);

    bool getParameters();

    private: // Main thread

public: // Function components
    bool run(sensor_msgs::PointCloud2& lidar_msg,
             sensor_msgs::Image& camera1_msg,
             sensor_msgs::Image& camera2_msg,
             sensor_msgs::Imu& imu_msg,
             sensor_msgs::NavSatFix& gps_msg,
             fs_msgs::GoSignal& go_signal_msg,
             fs_msgs::ControlCommand& control_command_msg,
             std_msgs::String& autonomous_mode_msg);
private:
    void getLidarPointCloud(sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);
    void getCameraImage(sensor_msgs::Image& msg, cv::Mat& image);
    void getImuData(sensor_msgs::Imu& msg, Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Quaterniond& orientation);
// Variables
private:
    bool is_initialized_;
    ros::NodeHandle pnh_;

public:

    // Perception
    std::shared_ptr<PerceptionParams> perception_params_;

    std::unique_ptr<RoiExtractor> roi_extractor_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_point_cloud_;

    std::unique_ptr<GroundRemoval> ground_removal_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_point_cloud_;
    
    std::unique_ptr<Clustering> clustering_;
    std::vector<Cone> cones_;
    
    std::unique_ptr<ColorDetection> color_detection_;
    cv::Mat projected_cones_image_;

    std::shared_ptr<LocalizationParams> localization_params_;
    std::unique_ptr<Localization> localization_;

    // Planning

    // Behavior planning

    // State machine
    std::unique_ptr<StateMachine> state_machine_;
    ASState planning_state_;

    // Local planning
    std::shared_ptr<TrajectoryParams> local_planning_params_;
    std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
    std::vector<TrajectoryPoint> trajectory_points_;   

    // Control
    std::shared_ptr<ControlParams> control_params_;

    std::unique_ptr<LateralController> lateral_controller_;
    std::unique_ptr<PIDController> longitudinal_controller_;
};


#endif // FORMULA_AUTONOMOUS_SYSTEM_HPP
