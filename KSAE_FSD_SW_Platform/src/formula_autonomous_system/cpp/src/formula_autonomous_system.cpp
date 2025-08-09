/**
 * @file formula_autonomous_system.cpp
 * @author Jiwon Seok (jiwonseok@hanyang.ac.kr)
 * @author MinKyu Cho (chomk2000@hanyang.ac.kr)
 * @brief 
 * @version 0.1
 * @date 2025-07-21
 * 
 * @copyright Copyright (c) 2025
 */

#include "formula_autonomous_system.hpp"

// ==================== Algorithm ====================
// Perception

// =================== RoiExtractor Implementation ===================

RoiExtractor::RoiExtractor(const std::shared_ptr<PerceptionParams> params)
    : params_(params) {
}

void RoiExtractor::extractRoi(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& roi_cloud) {
    roi_cloud->clear();
    roi_cloud->header = input_cloud->header;

    for (const auto& point : input_cloud->points) {
        // Check if point is within ROI
        if (point.x > params_->lidar_roi_x_min_ && point.x < params_->lidar_roi_x_max_ &&
            point.y > params_->lidar_roi_y_min_ && point.y < params_->lidar_roi_y_max_ &&
            point.z > params_->lidar_roi_z_min_ && point.z < params_->lidar_roi_z_max_) {
            roi_cloud->points.push_back(point);
        }
    }
}


// =================== GroundRemoval Implementation ===================

GroundRemoval::GroundRemoval(const std::shared_ptr<PerceptionParams> params) 
    : params_(params), rng_(std::random_device{}()) {
}

GroundRemoval::GroundRemoval(double distance_threshold, int max_iterations) 
    : rng_(std::random_device{}()) {
    // Create parameters from legacy constructor
    params_->lidar_ransac_distance_threshold_ = distance_threshold;
    params_->lidar_ransac_iterations_ = max_iterations;
}

void GroundRemoval::removeGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& non_ground_points) {
    
    ground_points->clear();
    non_ground_points->clear();
    
    if (input_cloud->empty()) return;
    
    Eigen::Vector4f best_plane; // ax + by + cz + d = 0
    std::vector<int> best_inliers;
    
    if (fitPlane(input_cloud, best_plane, best_inliers)) {
        // Extract ground and non-ground points
        std::vector<bool> is_ground(input_cloud->size(), false);
        for (int idx : best_inliers) {
            is_ground[idx] = true;
        }
        
        for (size_t i = 0; i < input_cloud->size(); ++i) {
            if (is_ground[i]) {
                ground_points->push_back(input_cloud->points[i]);
            } else {
                non_ground_points->push_back(input_cloud->points[i]);
            }
        }
    } else {
        // If plane fitting fails, treat all points as ground
        *ground_points = *input_cloud;
    }
    
    ground_points->header = input_cloud->header;
    non_ground_points->header = input_cloud->header;
}

bool GroundRemoval::fitPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    Eigen::Vector4f& plane_coefficients,
    std::vector<int>& inliers) {
    
    if (cloud->size() < 3) return false;
    
    // Initialize best inlier count
    int best_inlier_count = 0;
    std::uniform_int_distribution<int> dist(0, cloud->size() - 1);
    
    // RANSAC algorithm
    for (int iter = 0; iter < params_->lidar_ransac_iterations_; ++iter) {
        // Randomly select 3 points
        std::vector<int> sample_indices(3);
        for (int i = 0; i < 3; ++i) {
            sample_indices[i] = dist(rng_);
        }
        
        // Calculate plane from 3 points
        const auto& p1 = cloud->points[sample_indices[0]];
        const auto& p2 = cloud->points[sample_indices[1]];
        const auto& p3 = cloud->points[sample_indices[2]];
        
        // Calculate normal vector
        Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z); // Vector from p1 to p2
        Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z); // Vector from p1 to p3
        Eigen::Vector3f normal = v1.cross(v2);
        if (normal.norm() < 1e-6) continue; // Skip degenerate case
        normal.normalize();
        
        float d = -(normal.x() * p1.x + normal.y() * p1.y + normal.z() * p1.z);
        Eigen::Vector4f current_plane(normal.x(), normal.y(), normal.z(), d);
        
        // Count inliers
        std::vector<int> current_inliers;
        for (size_t i = 0; i < cloud->size(); ++i) {
            // d = - (normal.x * p1.x + normal.y * p1.y + normal.z * p1.z)
            if (pointToPlaneDistance(cloud->points[i], current_plane) < params_->lidar_ransac_distance_threshold_) {
                current_inliers.push_back(i);
            }
        }
        
        // Update best plane if current plane has more inliers
        if (current_inliers.size() > best_inlier_count) {
            best_inlier_count = current_inliers.size();
            plane_coefficients = current_plane;
            inliers = current_inliers;
        }
    }
    return best_inlier_count > 0; // If the number of inliers is greater than 0, return true
}

double GroundRemoval::pointToPlaneDistance(const pcl::PointXYZ& point, const Eigen::Vector4f& plane) {
    return std::abs(plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]);
}

// =================== Clustering Implementation ===================

Clustering::Clustering(const std::shared_ptr<PerceptionParams> params)
    : params_(params) {
    vehicle_to_lidar_transform_ = Eigen::Matrix4f::Identity();
    double x = params_->lidar_translation_[0];
    double y = params_->lidar_translation_[1];
    double z = params_->lidar_translation_[2];
    double roll = params_->lidar_rotation_[0];
    double pitch = params_->lidar_rotation_[1];
    double yaw = params_->lidar_rotation_[2];

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    // Create transformation matrix from vehicle base to lidar
    vehicle_to_lidar_transform_ << 
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x,
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y,
        -sp,   cp*sr,            cp*cr,            z,
        0,    0,                0,                1;
    
    std::cout << "Vehicle-to-Lidar transformation matrix:" << std::endl << vehicle_to_lidar_transform_ << std::endl;
}

Clustering::Clustering(double eps, int min_points, double min_cone_height, double max_cone_height) {
    // Create parameters from legacy constructor
    params_->lidar_dbscan_eps_ = eps;
    params_->lidar_dbscan_min_points_ = min_points;
    params_->lidar_cone_detection_min_height_ = min_cone_height;
    params_->lidar_cone_detection_max_height_ = max_cone_height;
    params_->lidar_cone_detection_min_points_ = 5;  // Default value
}

bool Clustering::extractCones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_points, std::vector<Cone>& cones) {
    if (input_points->empty()) return false;
    
    // Perform DBSCAN clustering: cluster = [cluster_indices]
    std::vector<std::vector<int>> clusters = dbscan(input_points);
    cones.clear();
    cones.reserve(clusters.size());
    
    // Extract cones from clusters
    for (const auto& cluster : clusters) {
        if (isValidCone(cluster, input_points)) {
            Cone cone;
            // Calculate centroid in lidar frame
            Eigen::Vector3f centroid = calculateCentroid(cluster, input_points);
            
            // Convert centroid to vehicle base frame
            Eigen::Vector4f centroid_4d(centroid.x(), centroid.y(), centroid.z(), 1.0f);
            Eigen::Vector4f centroid_lidar = vehicle_to_lidar_transform_ * centroid_4d;
            
            // Assign centroid to cone
            cone.center = pcl::PointXYZ(centroid_lidar.x(), centroid_lidar.y(), centroid_lidar.z());
            cone.color = "unknown"; // Initial color is unknown
            cone.confidence = std::min(1.0f, static_cast<float>(cluster.size()) / 50.0f);
            
            // Store cluster points
            for (int idx : cluster) {
                cone.points.push_back(input_points->points[idx]);
            }
            
            cones.push_back(cone);
        }
    }
    
    return true;
}

std::vector<std::vector<int>> Clustering::dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<std::vector<int>> clusters;
    std::vector<bool> visited(cloud->size(), false);
    std::vector<bool> clustered(cloud->size(), false);

    // Create KD-tree for efficient neighbor search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (visited[i]) continue;        // Skip if already visited
        visited[i] = true;
        std::vector<int> neighbors_indices = regionQuery(cloud, i, kdtree); // Find neighbors

        if (neighbors_indices.size() < params_->lidar_dbscan_min_points_) {
            continue; // Skip if less than min points
        }
        
        // Start new cluster
        std::vector<int> cluster;
        cluster.push_back(i);
        clustered[i] = true;
        // Expand cluster
        for (size_t j = 0; j < neighbors_indices.size(); ++j) {
            int neighbor_idx = neighbors_indices[j];
            
            if (!visited[neighbor_idx]) {
                visited[neighbor_idx] = true;
                std::vector<int> neighbor_neighbors = regionQuery(cloud, neighbor_idx, kdtree);
                // Add neighbor to cluster if it has more than min points
                if (neighbor_neighbors.size() >= params_->lidar_dbscan_min_points_) {
                    neighbors_indices.insert(neighbors_indices.end(), neighbor_neighbors.begin(), neighbor_neighbors.end());
                }
            }
            // Add neighbor to cluster if not already clustered
            if (!clustered[neighbor_idx]) {
                cluster.push_back(neighbor_idx);
                clustered[neighbor_idx] = true;
            }
        }
        
        clusters.push_back(cluster);
    }
    
    return clusters;
}

std::vector<int> Clustering::regionQuery(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int point_idx,
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree) {
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    kdtree->radiusSearch(point_idx, params_->lidar_dbscan_eps_, indices, distances);
    
    return indices;
}

bool Clustering::isValidCone(const std::vector<int>& cluster_indices, 
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    bool is_valid = true;
    if (cluster_indices.size() < params_->lidar_cone_detection_min_points_) 
        is_valid = false; // Too few points
    
    // Calculate height range
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();

    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (int idx : cluster_indices) {
        min_x = std::min(min_x, cloud->points[idx].x);
        max_x = std::max(max_x, cloud->points[idx].x);

        min_y = std::min(min_y, cloud->points[idx].y);
        max_y = std::max(max_y, cloud->points[idx].y);
    }
    
    float x_range = max_x - min_x;
    float y_range = max_y - min_y;

    float range = std::sqrt(x_range * x_range + y_range * y_range);

    if (range < params_->lidar_cone_detection_min_radius_ || range > params_->lidar_cone_detection_max_radius_)
        is_valid = false;

    return is_valid;
}

Eigen::Vector3f Clustering::calculateCentroid(const std::vector<int>& cluster_indices,
                                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    Eigen::Vector3f centroid;
    centroid.x() = centroid.y() = centroid.z() = 0.0f;
    
    for (int idx : cluster_indices) {
        centroid.x() += cloud->points[idx].x;
        centroid.y() += cloud->points[idx].y;
        centroid.z() += cloud->points[idx].z;
    }
    
    float size = static_cast<float>(cluster_indices.size());
    centroid.x() /= size;
    centroid.y() /= size;
    centroid.z() /= size;
    
    return centroid;
} 

// ==================== Color Detection ====================

ColorDetection::ColorDetection(const std::shared_ptr<PerceptionParams>& params)
    : params_(params) {
    // Initialize camera parameters from params
    initializeCameraParameters();
    
    // Compute camera to lidar transformation
    computeCameraToLidarTransform();
    
    std::cout << "ColorDetection initialized with camera-to-lidar transformation" << std::endl;
}

void ColorDetection::initializeCameraParameters() {
    // Create camera intrinsic matrix
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        params_->camera_fx_, 0, params_->camera_cx_,
        0, params_->camera_fy_, params_->camera_cy_,
        0, 0, 1);
    
    // Create distortion coefficients matrix
    std::cout << "Camera intrinsics initialized: fx=" << params_->camera_fx_ 
              << ", fy=" << params_->camera_fy_ 
              << ", cx=" << params_->camera_cx_ 
              << ", cy=" << params_->camera_cy_ << std::endl;
}

void ColorDetection::computeCameraToLidarTransform() {
    // Create transformation matrices from vehicle base to each sensor
    
    // Convert degrees to radians
    double deg_to_rad = CV_PI / 180.0;
    
    // Camera transformation (base -> camera)
    cv::Mat T_base_to_camera = createTransformationMatrix(
        params_->camera_translation_[0],  // x
        params_->camera_translation_[1],  // y
        params_->camera_translation_[2],  // z
        params_->camera_rotation_[0] * deg_to_rad,  // roll (deg -> rad)
        params_->camera_rotation_[1] * deg_to_rad,  // pitch (deg -> rad)
        params_->camera_rotation_[2] * deg_to_rad   // yaw (deg -> rad)
    );

    // Create transformation matrix for x-front, y-left, z-up to z-front, y-right, x-down frame
    cv::Mat T_x_front_to_z_front = createTransformationMatrix(
        0, 0, 0, -90.0 * deg_to_rad, 0, -90.0 * deg_to_rad
    );
    // T_base_to_camera = T_base_to_camera * T_x_front_to_z_front;

    // Compute camera to lidar transformation: camera -> vehicle * vehicle -> lidar = camera -> lidar
    cv::Mat T_camera_to_base = T_base_to_camera.inv();

    // Final transformation: camera -> lidar
    camera_to_base_rotation_ = T_camera_to_base.rowRange(0, 3).colRange(0, 3);
    camera_to_base_translation_ = T_camera_to_base.rowRange(0, 3).col(3);
    
    std::cout << "Camera-to-vehicle transformation computed:" << std::endl;
    std::cout << "  Rotation matrix:" << std::endl << camera_to_base_rotation_ << std::endl;
    std::cout << "  Translation vector:" << std::endl << camera_to_base_translation_ << std::endl;
}

cv::Mat ColorDetection::createTransformationMatrix(double x, double y, double z, double roll, double pitch, double yaw) {
    // Create transformation matrix from Euler angles (ZYX convention)
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    
    cv::Mat T = (cv::Mat_<double>(4, 4) <<
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x,
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y,
        -sp,   cp*sr,            cp*cr,            z,
        0,    0,                0,                1);
    
    return T;
}

std::string ColorDetection::detectConeColor(const Cone& cone, const cv::Mat& rgb_image) {
    if (rgb_image.empty()) {
        std::cerr << "Warning: Empty image provided for color detection" << std::endl;
        return "unknown";
    }
    
    // Project cone center to camera coordinates
    cv::Point2f projected_point = projectToCamera(cone.center);
    
    // Check if projection is within image bounds
    if (!isPointInImage(projected_point, rgb_image.size())) {
        return "out_of_image";
    }
    
    // Preprocess image if enabled
    cv::Mat processed_image = preprocessImage(rgb_image);
    
    // Convert to HSV for color analysis
    cv::Mat hsv_image;
    cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);
    
    // Analyze color in window around projected point
    int window_size = 20; // 20x20 pixel window
    ColorConfidence confidence = analyzeColorWindow(hsv_image, projected_point, window_size);

    // Select best color based on confidence
    std::string best_color = selectBestColor(confidence);
    
    return best_color;
}

std::vector<Cone> ColorDetection::classifyConesColor(const std::vector<Cone>& cones, const cv::Mat& rgb_image) {
    std::vector<Cone> classified_cones = cones;
    
    for (auto& cone : classified_cones) {
        cone.color = detectConeColor(cone, rgb_image);
    }
    
    return classified_cones;
}

cv::Point2f ColorDetection::projectToCamera(const pcl::PointXYZ& point_3d) {
    // Transform from Vehicle base to camera coordinate system
    cv::Mat cone_point_in_base = (cv::Mat_<double>(3, 1) << point_3d.x, point_3d.y, point_3d.z);
    cv::Mat cone_point_in_camera = camera_to_base_rotation_ * cone_point_in_base + camera_to_base_translation_;
    
    // Check if point is in front of camera
    if (cone_point_in_camera.at<double>(0, 0) <= 0) {
        return cv::Point2f(-1, -1); // Invalid projection
    }
    
    // Apply camera frame: from x-front, y-left, z-up to x-right, y-down, z-front
    double x_cam = -cone_point_in_camera.at<double>(1, 0);
    double y_cam = -cone_point_in_camera.at<double>(2, 0);
    double z_cam =  cone_point_in_camera.at<double>(0, 0);

    // Transform to image plane
    double x_img = x_cam / z_cam;
    double y_img = y_cam / z_cam;
    
    // Apply camera intrinsics for transform to pixel coordinates from image coordinates
    double u = params_->camera_fx_ * x_img + params_->camera_cx_;
    double v = params_->camera_fy_ * y_img + params_->camera_cy_;
    
    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

bool ColorDetection::isPointInImage(const cv::Point2f& point, const cv::Size& image_size) {
    return point.x >= 0 && point.x < image_size.width && 
           point.y >= 0 && point.y < image_size.height;
}

cv::Mat ColorDetection::visualizeProjection(const std::vector<Cone>& cones, const cv::Mat& rgb_image) {
    cv::Mat visualization = rgb_image.clone();
    
    for (const auto& cone : cones) {
        cv::Point2f projected = projectToCamera(cone.center);
        
        if (isPointInImage(projected, rgb_image.size())) {
            // Draw circle at projected position
            cv::Scalar color;
            if (cone.color == "yellow") {
                color = cv::Scalar(0, 255, 255); // Yellow in BGR
            } else if (cone.color == "blue") {
                color = cv::Scalar(255, 0, 0); // Blue in BGR
            } else if (cone.color == "orange") {
                color = cv::Scalar(0, 165, 255); // Orange in BGR
            } else {
                color = cv::Scalar(128, 128, 128); // Gray for unknown
            }
            
            cv::circle(visualization, projected, 10, color, 2);
            cv::putText(visualization, cone.color, 
                       cv::Point(projected.x + 15, projected.y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
    }
    
    return visualization;
}

ColorConfidence ColorDetection::analyzeColorWindow(const cv::Mat& hsv_image, const cv::Point2f& center, int window_size) {
    ColorConfidence confidence;
    
    // Get safe window around the projected point
    cv::Rect window = getSafeWindow(center, window_size, hsv_image.size());
    
    if (window.area() <= 0) {
        return confidence; // Return default (unknown = 1.0)
    }
    
    // Extract HSV region of interest
    cv::Mat hsv_roi = hsv_image(window);
    
    // Calculate confidence for each color
    confidence.yellow_confidence = static_cast<double>(countYellowPixels(hsv_roi));
    confidence.blue_confidence = static_cast<double>(countBluePixels(hsv_roi));
    confidence.orange_confidence = static_cast<double>(countOrangePixels(hsv_roi));
    
    return confidence;
}

int ColorDetection::countYellowPixels(const cv::Mat& hsv_roi) {
    int yellow_pixels = 0;
    
    for (int y = 0; y < hsv_roi.rows; ++y) {
        for (int x = 0; x < hsv_roi.cols; ++x) {
            cv::Vec3b hsv_pixel = hsv_roi.at<cv::Vec3b>(y, x);
            
            if (isInHSVRange(hsv_pixel, 
                           params_->camera_yellow_hue_min_, 
                           params_->camera_yellow_hue_max_,
                           params_->camera_yellow_sat_min_, 
                           params_->camera_yellow_val_min_)) {
                yellow_pixels++;
            }
        }
    }
    
    return yellow_pixels;
}

int ColorDetection::countBluePixels(const cv::Mat& hsv_roi) {
    int blue_pixels = 0;
    
    for (int y = 0; y < hsv_roi.rows; ++y) {
        for (int x = 0; x < hsv_roi.cols; ++x) {
            cv::Vec3b hsv_pixel = hsv_roi.at<cv::Vec3b>(y, x);
            
            if (isInHSVRange(hsv_pixel, 
                           params_->camera_blue_hue_min_, 
                           params_->camera_blue_hue_max_,
                           params_->camera_blue_sat_min_, 
                           params_->camera_blue_val_min_)) {
                blue_pixels++;
            }
        }
    }
    
    return blue_pixels;
}

int ColorDetection::countOrangePixels(const cv::Mat& hsv_roi) {
    int orange_pixels = 0;
    
    for (int y = 0; y < hsv_roi.rows; ++y) {
        for (int x = 0; x < hsv_roi.cols; ++x) {
            cv::Vec3b hsv_pixel = hsv_roi.at<cv::Vec3b>(y, x);
            
            if (isInHSVRange(hsv_pixel, 
                           params_->camera_orange_hue_min_, 
                           params_->camera_orange_hue_max_,
                           params_->camera_orange_sat_min_, 
                           params_->camera_orange_val_min_)) {
                orange_pixels++;
            }
        }
    }
    
    return orange_pixels;
}

std::string ColorDetection::selectBestColor(const ColorConfidence& confidence) {
    double max_confidence = 0.0;
    std::string best_color = "unknown";
    
    if (confidence.yellow_confidence > max_confidence) {
        max_confidence = confidence.yellow_confidence;
        best_color = "yellow";
    }
    
    if (confidence.blue_confidence > max_confidence) {
        max_confidence = confidence.blue_confidence;
        best_color = "blue";
    }
    
    if (confidence.orange_confidence > max_confidence) {
        max_confidence = confidence.orange_confidence;
        best_color = "orange";
    }
    
    // Require minimum confidence threshold to avoid false positives
    if (max_confidence < 0.001) {
        return "unknown";
    }
    
    return best_color;
}

cv::Mat ColorDetection::preprocessImage(const cv::Mat& rgb_image) {
    if (!params_->camera_enable_preprocessing_) {
        return rgb_image;
    }
    
    cv::Mat processed = rgb_image.clone();
    
    // Apply Gaussian blur for noise reduction
    if (params_->camera_gaussian_blur_sigma_ > 0) {
        int kernel_size = static_cast<int>(2 * params_->camera_gaussian_blur_sigma_ * 3 + 1);
        if (kernel_size % 2 == 0) kernel_size++;
        
        cv::GaussianBlur(processed, processed, 
                        cv::Size(kernel_size, kernel_size), 
                        params_->camera_gaussian_blur_sigma_);
    }
    
    // Apply bilateral filter for edge-preserving smoothing
    if (params_->camera_bilateral_filter_d_ > 0) {
        cv::Mat temp;
        cv::bilateralFilter(processed, temp, 
                           params_->camera_bilateral_filter_d_, 
                           80, 80);
        processed = temp;
    }
    
    return processed;
}

bool ColorDetection::isInHSVRange(const cv::Vec3b& hsv_pixel, int hue_min, int hue_max, int sat_min, int val_min) {
    int hue = hsv_pixel[0];
    int sat = hsv_pixel[1];
    int val = hsv_pixel[2];
    
    // Handle hue wraparound (e.g., red: 170-180 and 0-10)
    bool hue_in_range;
    if (hue_min <= hue_max) {
        hue_in_range = (hue >= hue_min && hue <= hue_max);
    } else {
        hue_in_range = (hue >= hue_min || hue <= hue_max);
    }
    
    return hue_in_range && sat >= sat_min && val >= val_min;
}

cv::Rect ColorDetection::getSafeWindow(const cv::Point2f& center, int window_size, const cv::Size& image_size) {
    int half_size = window_size / 2;
    
    // Select window left-top corner
    int x = static_cast<int>(center.x) - half_size;
    int y = static_cast<int>(center.y) - half_size;
    
    // Clamp to image boundaries
    x = std::max(0, std::min(x, image_size.width - window_size));
    y = std::max(0, std::min(y, image_size.height - window_size));
    
    // Select window width and height
    int width = std::min(window_size, image_size.width - x);
    int height = std::min(window_size, image_size.height - y);
    
    // Return window [left, top, width, height]
    return cv::Rect(x, y, width, height);
} 

// ==================== Localization ====================

Localization::Localization(const std::shared_ptr<LocalizationParams>& params)
    : params_(params),
      state_(8, 0.0),
      last_time_sec_(0.0),
      ref_wgs84_position_(0.0, 0.0),
      prev_gps_enu_(0.0, 0.0),
      prev_gps_time_sec_(0.0)
{
    // Set reference GPS position
    if (params_->use_user_defined_ref_wgs84_position_ == true) {
        ref_wgs84_position_ << params_->ref_wgs84_latitude_, params_->ref_wgs84_longitude_;
    }
    else {
        ref_wgs84_position_ << 0.0, 0.0;
    }
    
    // Initialize state vector [x, y, yaw, vx, vy, yawrate, ax, ay]
    state_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}


/**
 * @brief Update with IMU data (acceleration, yaw rate and IMU orientation)
 * @param imu_input [ax, ay, yaw_rate]
 * @param q_imu IMU orientation
 * @param curr_time_sec Current time in seconds
 */
void Localization::updateImu(const Eigen::Vector3d& imu_input, Eigen::Quaterniond& q_imu, double curr_time_sec) {
    
    // Update state
    state_[5] = imu_input[2]; // yaw rate
    state_[6] = imu_input[0]; // acceleration
    state_[7] = imu_input[1]; // lateral acceleration

    // Update yaw angle
    double yaw_angle = atan2(2.0 * (q_imu.w() * q_imu.z() + q_imu.x() * q_imu.y()), 1.0 - 2.0 * (q_imu.y() * q_imu.y() + q_imu.z() * q_imu.z()));
    state_[2] = yaw_angle; // yaw angle
    
    // Update timing
    double dt = curr_time_sec - last_time_sec_;
    if (dt > 0.0 && last_time_sec_ > std::numeric_limits<double>::epsilon()) {
        // Predict if time is not updated
        state_ = predictState(state_, dt);
    }
    last_time_sec_ = curr_time_sec;
}

void Localization::updateGps(const Eigen::Vector2d& gps_wgs84, double curr_time_sec) {
    if (params_->use_user_defined_ref_wgs84_position_ == false) {
        if (ref_wgs84_position_.x() == 0.0 && ref_wgs84_position_.y() == 0.0) {
            ref_wgs84_position_ << gps_wgs84[0], gps_wgs84[1];
        }
    }

    // Update timing
    double dt = curr_time_sec - last_time_sec_;
    if (dt > 0.0 && last_time_sec_ > std::numeric_limits<double>::epsilon()) {
        // Predict if time is not updated
        state_ = predictState(state_, dt);
        last_time_sec_ = curr_time_sec;
    }
    
    // Convert GPS to ENU coordinates
    Eigen::Vector2d gps_enu = wgs84ToEnu(gps_wgs84);
    
    // Update step with GPS measurement
    state_[0] = gps_enu[0]; // x
    state_[1] = gps_enu[1]; // y

    // Update velocity
    double dt_gps = curr_time_sec - prev_gps_time_sec_;
    if (dt_gps > 0.0 && curr_time_sec > std::numeric_limits<double>::epsilon()) {
        double dx = state_[0] - prev_gps_enu_[0];
        double dy = state_[1] - prev_gps_enu_[1];
        prev_gps_enu_ << state_[0], state_[1];
        double vx = dx / dt_gps;
        double vy = dy / dt_gps;
        state_[3] = params_->alpha_velocity_ * state_[3] + (1-params_->alpha_velocity_) * (vx * cos(-state_[2]) - vy * sin(-state_[2])); // longitudinal velocity
        state_[4] = params_->alpha_velocity_ * state_[4] + (1-params_->alpha_velocity_) * (vx * sin(-state_[2]) + vy * cos(-state_[2])); // lateral velocity
        // Update previous GPS position
        prev_gps_enu_ << state_[0], state_[1];
    }
    prev_gps_time_sec_ = curr_time_sec;
}

std::vector<double> Localization::predictState(const std::vector<double>& state, double dt) {
    double x = state[0];
    double y = state[1];
    double yaw = state[2];
    double vx = state[3];
    double vy = state[4];
    double yaw_rate = state[5];
    double ax = state[6];
    double ay = state[7];

    double yaw_middle = yaw + yaw_rate * dt * 0.5;

    double new_x = x + vx * cos(yaw_middle) * dt + 0.5 * ax * cos(yaw_middle) * dt * dt;
    double new_y = y + vx * sin(yaw_middle) * dt + 0.5 * ax * sin(yaw_middle) * dt * dt;
    double new_yaw = yaw + yaw_rate * dt;
    double new_vx = vx + ax * dt;
    double new_vy = vy + ay * dt;

    std::vector<double> new_state = {new_x, new_y, new_yaw, new_vx, new_vy, yaw_rate, ax, ay};

    return new_state;
}

Eigen::Vector2d Localization::wgs84ToEnu(const Eigen::Vector2d& wgs84_pos) const {
    // Convert WGS84 (lat, lon) to ENU coordinates
    double lat_rad = wgs84_pos[0] * DEG_TO_RAD;
    double lon_rad = wgs84_pos[1] * DEG_TO_RAD;
    double ref_lat_rad = ref_wgs84_position_[0] * DEG_TO_RAD;
    double ref_lon_rad = ref_wgs84_position_[1] * DEG_TO_RAD;
    
    double dlat = lat_rad - ref_lat_rad;
    double dlon = lon_rad - ref_lon_rad;
    
    // Convert WGS84 to ENU coordinates: x = (lon - ref_lon) * EARTH_RADIUS * cos(ref_lat), y = (lat - ref_lat) * EARTH_RADIUS
    double x = EARTH_RADIUS * dlon * cos(ref_lat_rad); // cos(ref_lat_rad) to account for latitude scaling
    double y = EARTH_RADIUS * dlat;
    
    return Eigen::Vector2d(x, y);
}

// ==================== Planning ====================


StateMachine::StateMachine()
    : current_state_(ASState::AS_OFF)
    , previous_state_(ASState::AS_OFF)
    , state_entry_time_(std::chrono::steady_clock::now())
    , last_update_time_(std::chrono::steady_clock::now())
    , current_mission_("")
    , mission_track_("")
    , mission_active_(false)
{
    // Initialize valid state transitions
    initializeValidTransitions();
    
    std::cout << "StateMachine: Initialized in AS_OFF state" << std::endl;
}

void StateMachine::initializeValidTransitions() {
    valid_transitions_.clear();
    
    // AS_OFF transitions
    valid_transitions_[{ASState::AS_OFF, ASState::AS_READY}] = true;
    
    // AS_READY transitions
    valid_transitions_[{ASState::AS_READY, ASState::AS_DRIVING}] = true;
    valid_transitions_[{ASState::AS_READY, ASState::AS_OFF}] = true;
    
    // AS_DRIVING transitions
    valid_transitions_[{ASState::AS_DRIVING, ASState::AS_OFF}] = true;
}

bool StateMachine::isValidTransition(ASState from, ASState to) const {
    auto it = valid_transitions_.find({from, to});
    return it != valid_transitions_.end() && it->second;
}

StateTransitionResult StateMachine::processEvent(ASEvent event) {
    ASState target_state = current_state_;
    std::string reason = eventToString(event);
    
    switch (event) {
        case ASEvent::SYSTEM_INIT:
            if (current_state_ == ASState::AS_OFF) {
                target_state = ASState::AS_READY;
            }
            break;
            
        case ASEvent::GO_SIGNAL:
            if (current_state_ == ASState::AS_READY) {
                target_state = ASState::AS_DRIVING;
                mission_active_ = true;
            }
            break;
        default:
            return StateTransitionResult(false, current_state_, current_state_, 
                                       "Unknown event: " + reason);
    }
    
    if (target_state != current_state_) {
        if (performStateTransition(target_state, reason)) {
            return StateTransitionResult(true, previous_state_, current_state_, reason);
        } else {
            return StateTransitionResult(false, current_state_, current_state_, 
                                       "Transition failed: " + reason);
        }
    }
    
    return StateTransitionResult(true, current_state_, current_state_, "No transition needed");
}

bool StateMachine::performStateTransition(ASState new_state, const std::string& reason) {
    if (!isValidTransition(current_state_, new_state)) {
        std::cout << "StateMachine: Invalid transition from " << stateToString(current_state_) 
                  << " to " << stateToString(new_state) << std::endl;
        return false;
    }
    
    // Exit current state
    bool exit_success = true;
    switch (current_state_) {
        case ASState::AS_OFF: exit_success = exitAS_OFF(); break;
        case ASState::AS_READY: exit_success = exitAS_READY(); break;
        case ASState::AS_DRIVING: exit_success = exitAS_DRIVING(); break;
    }
    
    if (!exit_success) {
        std::cout << "StateMachine: Failed to exit state " << stateToString(current_state_) << std::endl;
        return false;
    }
    
    // Update state
    previous_state_ = current_state_;
    current_state_ = new_state;
    state_entry_time_ = std::chrono::steady_clock::now();
    
    // Enter new state
    bool enter_success = true;
    switch (new_state) {
        case ASState::AS_OFF: enter_success = enterAS_OFF(); break;
        case ASState::AS_READY: enter_success = enterAS_READY(); break;
        case ASState::AS_DRIVING: enter_success = enterAS_DRIVING(); break;
    }
    
    logStateTransition(previous_state_, current_state_, reason);
    
    return enter_success;
}

// State entry functions
bool StateMachine::enterAS_OFF() {
    std::cout << "StateMachine: Entering AS_OFF state" << std::endl;
    mission_active_ = false;
    return true;
}

bool StateMachine::enterAS_READY() {
    std::cout << "StateMachine: Entering AS_READY state" << std::endl;
    return true;
}

bool StateMachine::enterAS_DRIVING() {
    std::cout << "StateMachine: Entering AS_DRIVING state" << std::endl;
    mission_active_ = true;
    return true;
}

// State exit functions
bool StateMachine::exitAS_OFF() { return true; }
bool StateMachine::exitAS_READY() { return true; }
bool StateMachine::exitAS_DRIVING() { return true; }

void StateMachine::injectSystemInit() {
    processEvent(ASEvent::SYSTEM_INIT);
}

void StateMachine::injectGoSignal(const std::string& mission, const std::string& track) {
    current_mission_ = mission;
    mission_track_ = track;
    processEvent(ASEvent::GO_SIGNAL);
}

void StateMachine::printStateInfo() const {
    std::cout << "=== State Machine Status ===" << std::endl;
    std::cout << "Current State: " << getCurrentStateString() << std::endl;
    std::cout << "Previous State: " << stateToString(previous_state_) << std::endl;
    std::cout << "Time in State: " << getTimeInCurrentState() << " seconds" << std::endl;
    std::cout << "Mission: " << current_mission_ << " (Active: " << (mission_active_ ? "Yes" : "No") << ")" << std::endl;
    std::cout << "=========================" << std::endl;
}

double StateMachine::getTimeInCurrentState() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_entry_time_);
    return elapsed.count() / 1000.0;
}

std::string StateMachine::stateToString(ASState state) const {
    switch (state) {
        case ASState::AS_OFF: return "AS_OFF";
        case ASState::AS_READY: return "AS_READY";
        case ASState::AS_DRIVING: return "AS_DRIVING";
        default: return "UNKNOWN";
    }
}

std::string StateMachine::eventToString(ASEvent event) const {
    switch (event) {
        case ASEvent::SYSTEM_INIT: return "SYSTEM_INIT";
        case ASEvent::SYSTEM_READY: return "SYSTEM_READY";
        case ASEvent::GO_SIGNAL: return "GO_SIGNAL";
        default: return "UNKNOWN_EVENT";
    }
}

void StateMachine::logStateTransition(ASState from, ASState to, const std::string& reason) {
    std::cout << "StateMachine: " << stateToString(from) << " -> " << stateToString(to) 
              << " (Reason: " << reason << ")" << std::endl;
} 

// ==================== Trajectory Generator ====================


TrajectoryGenerator::TrajectoryGenerator(const std::shared_ptr<TrajectoryParams>& params)
    : params_(params)
    , generated_trajectories_(0)
    , average_generation_time_(0.0)
    , last_generation_time_(0.0)
{
    std::cout << "TrajectoryGenerator: Initialized with lookahead " 
              << params_->lookahead_distance_ << "m, spacing " 
              << params_->waypoint_spacing_ << "m" << std::endl;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(const std::vector<Cone>& cones, ASState planning_state)
{
    if(planning_state == ASState::AS_DRIVING){
        auto trajectory = generateConesTrajectory(cones);
        return trajectory;
    }
    else {
        auto trajectory = generateStopTrajectory();
        return trajectory;
    }

}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateConesTrajectory(const std::vector<Cone>& cones)
{
    last_trajectory_.clear();
    
    // 1. 입력 콘들이 이미 로컬 좌표계 (차량 중심이 원점, 전방이 x축)
    std::vector<Eigen::Vector2d> blue_cones_local, yellow_cones_local;
    
    for (const auto& cone : cones) {
        Eigen::Vector2d cone_pos(cone.center.x, cone.center.y);
        
        // 전방에 있는 콘만 고려
        if (cone_pos.x() > 0.0) {
            if (cone.color == "blue") {
                blue_cones_local.push_back(cone_pos);
            } else if (cone.color == "yellow") {
                yellow_cones_local.push_back(cone_pos);
            }
        }
    }
    
    if (blue_cones_local.empty() && yellow_cones_local.empty()) {
        // 콘이 없으면 직진 경로 생성
        int num_points = static_cast<int>(params_->lookahead_distance_ / params_->waypoint_spacing_) + 1;
        for (int i = 0; i < num_points; ++i) {
            double x = i * params_->waypoint_spacing_;
            last_trajectory_.emplace_back(x, 0.0, 0.0, 0.0, params_->default_speed_, x);
        }
        return last_trajectory_;
    }
    
    // 2. 로컬 좌표계에서 경로 생성
    std::vector<Eigen::Vector2d> path_points;
    path_points.push_back(Eigen::Vector2d(0.0, 0.0)); // 차량 위치 (원점)
    
    // 전방 지점들에 대해 중앙점 계산
    int num_points = static_cast<int>(params_->lookahead_distance_ / params_->waypoint_spacing_);
    for (int i = 1; i <= num_points; ++i) {
        double target_x = i * params_->waypoint_spacing_;
        
        // 해당 전방 거리에서 가장 가까운 파란 콘과 노란 콘 찾기
        Eigen::Vector2d* closest_blue = nullptr;
        Eigen::Vector2d* closest_yellow = nullptr;
        double min_blue_dist = 999.0, min_yellow_dist = 999.0;
        
        // 파란 콘 찾기
        for (auto& cone_pos : blue_cones_local) {
            double dist = std::abs(cone_pos.x() - target_x);
            if (dist < min_blue_dist && cone_pos.x() > 0.0 && cone_pos.x() < params_->max_cone_distance_) {
                min_blue_dist = dist;
                closest_blue = &cone_pos;
            }
        }
        
        // 노란 콘 찾기
        for (auto& cone_pos : yellow_cones_local) {
            double dist = std::abs(cone_pos.x() - target_x);
            if (dist < min_yellow_dist && cone_pos.x() > 0.0 && cone_pos.x() < params_->max_cone_distance_) {
                min_yellow_dist = dist;
                closest_yellow = &cone_pos;
            }
        }
        
        // 경로점 계산 (로컬 좌표계)
        Eigen::Vector2d waypoint;
        if (closest_blue && closest_yellow) {
            // 양쪽 콘이 있으면 중앙점 (safety_margin_ 고려 가능)
            waypoint.x() = target_x;
            waypoint.y() = (closest_blue->y() + closest_yellow->y()) * 0.5;
        } else if (closest_blue) {
            // 파란 콘(좌측)만 있으면 우측으로 lane_offset만큼 떨어진 점
            waypoint.x() = target_x;
            waypoint.y() = closest_blue->y() - params_->lane_offset_; // 우측
        } else if (closest_yellow) {
            // 노란 콘(우측)만 있으면 좌측으로 lane_offset만큼 떨어진 점
            waypoint.x() = target_x;
            waypoint.y() = closest_yellow->y() + params_->lane_offset_; // 좌측
        } else {
            // 콘이 없으면 직진
            waypoint.x() = target_x;
            waypoint.y() = 0.0;
        }
        
        path_points.push_back(waypoint);
    }
    
    // 3. 로컬 좌표계에서 궤적점 생성
    for (size_t i = 0; i < path_points.size(); ++i) {
        double local_yaw = 0.0;
        if (i < path_points.size() - 1) {
            Eigen::Vector2d dir = path_points[i + 1] - path_points[i];
            local_yaw = atan2(dir.y(), dir.x());
        } else if (i > 0) {
            Eigen::Vector2d dir = path_points[i] - path_points[i - 1];
            local_yaw = atan2(dir.y(), dir.x());
        }
        
        last_trajectory_.emplace_back(
            path_points[i].x(), path_points[i].y(), 
            local_yaw, 0.0, params_->default_speed_, path_points[i].x()
        );
    }
    
    return last_trajectory_;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateStopTrajectory()
{
    last_trajectory_.clear();
    // 전방 지점들에 대해 중앙점 계산
    int num_points = static_cast<int>(params_->lookahead_distance_ / params_->waypoint_spacing_);
    for (int i = 0; i < num_points; ++i) {
        double x = i * params_->waypoint_spacing_;
        last_trajectory_.emplace_back(x, 0.0, 0.0, 0.0, 0.0, x);
    }
    return last_trajectory_;
}

// Utility functions
double TrajectoryGenerator::calculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const
{
    return (p1 - p2).norm();
}

double TrajectoryGenerator::calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const
{
    Eigen::Vector2d diff = p2 - p1;
    return std::atan2(diff.y(), diff.x());
}

void TrajectoryGenerator::printTrajectoryStats() const
{
    std::cout << "=== Trajectory Generator Statistics ===" << std::endl;
    std::cout << "Generated trajectories: " << generated_trajectories_ << std::endl;
    std::cout << "Last generation time: " << last_generation_time_ << " ms" << std::endl;
    std::cout << "Average generation time: " << average_generation_time_ << " ms" << std::endl;
    std::cout << "Last trajectory points: " << last_trajectory_.size() << std::endl;
    
    if (!last_trajectory_.empty()) {
        std::cout << "Trajectory length: " << last_trajectory_.back().s << " m" << std::endl;
        
        double avg_speed = 0.0;
        double max_curvature = 0.0;
        for (const auto& point : last_trajectory_) {
            avg_speed += point.speed;
            max_curvature = std::max(max_curvature, point.curvature);
        }
        avg_speed /= last_trajectory_.size();
        
        std::cout << "Average speed: " << avg_speed << " m/s" << std::endl;
        std::cout << "Maximum curvature: " << max_curvature << " (1/m)" << std::endl;
    }
    std::cout << "=====================================" << std::endl;
} 

// ==================== Control ====================

// Lateral Control
PurePursuit::PurePursuit(const std::shared_ptr<ControlParams>& params)
    : params_(params) {
}

double PurePursuit::calculateSteeringAngle(const VehicleState& current_state, const std::vector<TrajectoryPoint>& path) const {
    if (path.empty()) {
        return 0.0;
    }

    int target_idx = findTargetPointIndex(path);
    const Eigen::Vector2d& target_point = path[target_idx].position;

    return calculateSteeringAngleInternal(target_point);
}

int PurePursuit::findTargetPointIndex(const std::vector<TrajectoryPoint>& path) const {
    int target_idx = 0;
    for (int i = 0; i < path.size(); ++i) {
        int current_idx = i % path.size();
        double dist_from_car = path[current_idx].position.norm();
        
        if (dist_from_car >= params_->pp_lookahead_distance_) {
            target_idx = current_idx;
            break;
        }
    }
    return target_idx;
}

double PurePursuit::calculateSteeringAngleInternal(const Eigen::Vector2d& target_point) const {
    double alpha = std::atan2(target_point.y(), target_point.x() + params_->vehicle_length_ * 0.5); // local trajectory is defined at the center of the vehicle
    double delta = std::atan2(2.0 * params_->vehicle_length_ * std::sin(alpha), params_->pp_lookahead_distance_);
    
    return std::clamp(delta, -params_->pp_max_steer_angle_, params_->pp_max_steer_angle_);
}

// Stanley Controller
Stanley::Stanley(const std::shared_ptr<ControlParams>& params) : params_(params) {}

double Stanley::calculateSteeringAngle(const VehicleState& current_state, const std::vector<TrajectoryPoint>& path) const
{
    if (path.empty()) return 0.0;

    // Get the point of front axle
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = 0;
    Eigen::Vector2d front_axle_pos = path[0].position + 
        Eigen::Vector2d(params_->vehicle_length_ * cos(path[0].yaw) * 0.5, 
                        params_->vehicle_length_ * sin(path[0].yaw) * 0.5); // local trajectory is defined at the center of the vehicle
    
    // Find the closest point on the path to the front axle
    for (int i = 0; i < path.size(); ++i) {
        double dist = (path[i].position - front_axle_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Get the target point
    const TrajectoryPoint& target_point = path[closest_idx];
    double path_yaw = target_point.yaw;

    // Calculate the cross track error
    Eigen::Vector2d error_vec = front_axle_pos - target_point.position;
    double cross_track_error = error_vec.norm();
    
    // Calculate the heading error
    double path_dx = cos(path_yaw + M_PI_2);
    double path_dy = sin(path_yaw + M_PI_2);
    if (error_vec.dot(Eigen::Vector2d(path_dx, path_dy)) < 0) {
        cross_track_error *= -1.0;
    }

    double heading_error = path_yaw;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // Calculate the cross track steering, 0.1 is added to the speed to avoid division by zero
    double cross_track_steering = atan2(params_->stanley_k_gain_ * cross_track_error, current_state.speed + 0.1);

    // Calculate the steering angle
    double steering_angle = heading_error + cross_track_steering;

    // Clamp the steering angle: -max_steer_angle_ <= steering_angle <= max_steer_angle_
    return std::clamp(steering_angle, -params_->pp_max_steer_angle_, params_->pp_max_steer_angle_);
}

// Longitudinal Control

PIDController::PIDController(const std::shared_ptr<ControlParams>& params)
    : params_(params),
      kp_(params_->pid_kp_), ki_(params_->pid_ki_), kd_(params_->pid_kd_), 
      min_output_(0.0), max_output_(params_->max_throttle_),
      integral_error_(0.0), previous_error_(0.0), first_run_(true) {}

double PIDController::calculate(double setpoint, double measured_value) {
    auto current_time = std::chrono::steady_clock::now();
    
    // 첫 실행 시 dt가 비정상적으로 커지는 것을 방지
    if (first_run_) {
        last_time_ = current_time;
        first_run_ = false;
        previous_error_ = setpoint - measured_value;
        return 0.0;
    }

    // 시간 변화량(dt) 계산
    std::chrono::duration<double> delta_time = current_time - last_time_;
    double dt = delta_time.count();
    
    // dt가 0이거나 너무 작은 경우, 계산 오류를 방지
    if (dt <= 1e-6) {
        // 이전 제어값을 그대로 사용하거나 0을 반환할 수 있습니다.
        // 여기서는 P, I, D 중 P항만 계산하여 반환합니다.
        return std::clamp(kp_ * (setpoint - measured_value), min_output_, max_output_);
    }

    // 1. 비례(Proportional) 항 계산
    double error = setpoint - measured_value;
    double p_term = kp_ * error;

    // 2. 적분(Integral) 항 계산
    integral_error_ += error * dt;
    // Integral Wind-up 방지를 위해 적분항도 제한할 수 있습니다. (선택적)
    // integral_error_ = std::clamp(integral_error_, min_integral, max_integral);
    double i_term = ki_ * integral_error_;

    // 3. 미분(Derivative) 항 계산
    double derivative_error = (error - previous_error_) / dt;
    double d_term = kd_ * derivative_error;

    // 최종 제어 출력값 계산
    double output = p_term + i_term + d_term;

    // 다음 계산을 위해 현재 상태 저장
    previous_error_ = error;
    last_time_ = current_time;

    // 출력값을 지정된 범위 내로 제한(clamping)
    return std::clamp(output, min_output_, max_output_);
}

void PIDController::reset() {
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    first_run_ = true;
}

// ==================== FormulaAutonomousSystem ====================

FormulaAutonomousSystem::FormulaAutonomousSystem():
    is_initialized_(false),
    pnh_(ros::NodeHandle()),
    ground_removal_(nullptr),
    clustering_(nullptr),
    color_detection_(nullptr),
    localization_(nullptr),
    state_machine_(nullptr),
    lateral_controller_(nullptr),
    longitudinal_controller_(nullptr) {
}

FormulaAutonomousSystem::~FormulaAutonomousSystem(){
    ROS_INFO("FormulaAutonomousSystem: Destructor called");
    return;
}

// init 함수는 ros node 초기화시 시행되므로 작성 필요.
bool FormulaAutonomousSystem::init(ros::NodeHandle& pnh){
    pnh_ = pnh;

    // Get parameters
    perception_params_ = std::make_shared<PerceptionParams>();
    localization_params_ = std::make_shared<LocalizationParams>();
    local_planning_params_ = std::make_shared<TrajectoryParams>();
    control_params_ = std::make_shared<ControlParams>();

    // Get parameters
    if (!getParameters()) {
        ROS_ERROR("FormulaAutonomousSystem: Failed to get parameters");
        return false;
    }

    // Algorithms

    // Perception
    // LiDAR perception
    roi_extractor_ = std::make_unique<RoiExtractor>(perception_params_);
    ground_removal_ = std::make_unique<GroundRemoval>(perception_params_);
    clustering_ = std::make_unique<Clustering>(perception_params_);

    roi_point_cloud_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    ground_point_cloud_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    non_ground_point_cloud_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

    // Camera perception
    color_detection_ = std::make_unique<ColorDetection>(perception_params_);

    // Localization
    localization_ = std::make_unique<Localization>(localization_params_);

    // Planning
    state_machine_ = std::make_unique<StateMachine>();

    // Local planning
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>(local_planning_params_);

    // Control
    if (control_params_->lateral_controller_type_ == "Stanley") { 
        lateral_controller_ = std::make_unique<Stanley>(control_params_);
        ROS_INFO("Lateral Controller: Stanley selected");
    } else { // Default to Pure Pursuit
        lateral_controller_ = std::make_unique<PurePursuit>(control_params_);
        ROS_INFO("Lateral Controller: PurePursuit selected");
    }
    longitudinal_controller_ = std::make_unique<PIDController>(control_params_);

    is_initialized_ = true;
    return true;
}

bool FormulaAutonomousSystem::getParameters(){
    // Get parameters
    if(perception_params_->getParameters(pnh_) == false){
        ROS_ERROR("FormulaAutonomousSystem: Failed to get perception parameters");
        return false;
    }
    if(localization_params_->getParameters(pnh_) == false){
        ROS_ERROR("FormulaAutonomousSystem: Failed to get localization parameters");
        return false;
    }
    if(local_planning_params_->getParameters(pnh_) == false){
        ROS_ERROR("FormulaAutonomousSystem: Failed to get local planning parameters");
        return false;
    }
    if(control_params_->getParameters(pnh_) == false){
        ROS_ERROR("FormulaAutonomousSystem: Failed to get control parameters");
        return false;
    }
    return true;
}

/**
 * @brief Run the formula autonomous system
 * @param lidar_msg: (input) Lidar point cloud
 * @param camera1_msg: (input) Camera1 image
 * @param camera2_msg: (input) Camera2 image
 * @param imu_msg: (input) IMU data
 * @param gps_msg: (input) GPS data
 * @param go_signal_msg: (input) Go signal
 * @param control_command_msg: (output) Control command
 * @return true if the formula autonomous system is running, false otherwise
 */
bool FormulaAutonomousSystem::run(sensor_msgs::PointCloud2& lidar_msg,
                                    sensor_msgs::Image& camera1_msg,
                                    sensor_msgs::Image& camera2_msg,
                                    sensor_msgs::Imu& imu_msg,
                                    sensor_msgs::NavSatFix& gps_msg,
                                    fs_msgs::GoSignal& go_signal_msg,
                                    fs_msgs::ControlCommand& control_command_msg,
                                    std_msgs::String& autonomous_mode_msg){

    if(is_initialized_ == false){
        static int non_init_count = 0;
        non_init_count++;
        if(non_init_count % 1000 == 0){
            ROS_WARN("FormulaAutonomousSystem: Not initialized");
        }
        return false;
    }

    // State machine: System initialization
    state_machine_->injectSystemInit();

    // Point cloud Update
    bool cone_updated = false;
    // Perception
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    getLidarPointCloud(lidar_msg, lidar_point_cloud);

    // Roi extraction
    roi_extractor_->extractRoi(lidar_point_cloud, roi_point_cloud_);

    // Ground removal
    ground_removal_->removeGround(roi_point_cloud_, ground_point_cloud_, non_ground_point_cloud_);

    // Clustering
    clustering_->extractCones(non_ground_point_cloud_, cones_);

    // Color detection
    cv::Mat camera1_image;
    getCameraImage(camera1_msg, camera1_image);
    cones_ = color_detection_->classifyConesColor(cones_, camera1_image);
    projected_cones_image_ = color_detection_->visualizeProjection(cones_, camera1_image);  // For Debugging

    // Localization
    Eigen::Vector3d acc;    
    Eigen::Vector3d gyro;
    Eigen::Quaterniond orientation;
    getImuData(imu_msg, acc, gyro, orientation);
    localization_->updateImu(Eigen::Vector3d(acc.x(), acc.y(), gyro.z()), orientation, imu_msg.header.stamp.toSec());
    localization_->updateGps(Eigen::Vector2d(gps_msg.latitude, gps_msg.longitude), gps_msg.header.stamp.toSec());

    // Go signal Update
    // State machine: Go signal
    if(go_signal_msg.mission != "None" && go_signal_msg.mission != ""){
        state_machine_->injectGoSignal(go_signal_msg.mission, go_signal_msg.track);
    }

    // Planning
    planning_state_ = state_machine_->getCurrentState();

    // Local planning: Trajectory generator
    trajectory_points_ = trajectory_generator_->generateTrajectory(cones_, planning_state_);
    
    // Control
    // 1. 측위 모듈로부터 현재 차량 상태를 가져옵니다.
    auto current_pose = localization_->getCurrentPose(); // return type: Eigen::Vector3d(x, y, yaw)
    auto current_vel = localization_->getCurrentVelocity(); // 이 함수가 속력(double)을 반환함
    VehicleState vehicle_state(current_pose.x(), current_pose.y(), current_pose.z(), current_vel);

    // 2. 횡방향 제어: 경로와 현재 상태를 기반으로 조향각 계산
    double steering_angle = lateral_controller_->calculateSteeringAngle(vehicle_state, trajectory_points_);

    // 3. 종방향 제어: 목표 속도와 현재 속도를 기반으로 스로틀 계산
    double throttle = longitudinal_controller_->calculate(trajectory_points_[0].speed, vehicle_state.speed);

    // 4. 계산된 제어 명령을 멤버 변수에 저장
    control_command_msg.steering = -steering_angle; // FSDS 좌표계에 맞게 음수(-) 적용
    if (throttle > 0.0){
        control_command_msg.throttle = throttle;
        control_command_msg.brake = 0.0;
    }
    else{
        control_command_msg.throttle = 0.0;
        control_command_msg.brake = -throttle;
    }

    // State machine: Autonomous mode
    std::string autonomous_mode = state_machine_->getCurrentStateString();
    autonomous_mode_msg.data = autonomous_mode;
    
    // Debug
    static int count = 0;
    count++;
    if(count % 10 == 0){
        static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = current_time - last_time;
        double fps = (duration.count() > 0.0) ? (10.0 / duration.count()) : 0.0;
        std::cout << "count: " << count << ", \tavg time: " << duration.count()*1000 / 10 << "ms, \tavg FPS: " << fps << " Hz" << std::endl;
        last_time = current_time;
    }
    if(count > 1000){
        count = 0;
    }

    return true;
}

void FormulaAutonomousSystem::getLidarPointCloud(sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud){
    // Convert ROS PointCloud2 message to PCL point cloud
    pcl::fromROSMsg(msg, *point_cloud);
    return;
}

void FormulaAutonomousSystem::getCameraImage(sensor_msgs::Image& msg, cv::Mat& image){
    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Convert ROS message to cv_bridge format
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // Extract OpenCV Mat from cv_bridge
        image = cv_ptr->image.clone();
        
        // Check if image is valid
        if (image.empty()) {
            ROS_WARN("Received empty camera image");
            return;
        }
        
        // Optional: Log image dimensions for debugging
        ROS_DEBUG("Camera image received: %dx%d, channels: %d", 
                  image.cols, image.rows, image.channels());
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception while converting camera image: %s", e.what());
        // Initialize empty image on error
        image = cv::Mat();
    }
    return;
}

void FormulaAutonomousSystem::getImuData(sensor_msgs::Imu& msg, Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Quaterniond& orientation){
    // Extract IMU orientation data from ROS message
    orientation.w() = msg.orientation.w;
    orientation.x() = msg.orientation.x;
    orientation.y() = msg.orientation.y;
    orientation.z() = msg.orientation.z;
    
    // Extract raw acceleration data
    Eigen::Vector3d raw_acc;
    raw_acc.x() = msg.linear_acceleration.x;
    raw_acc.y() = msg.linear_acceleration.y;
    raw_acc.z() = msg.linear_acceleration.z;
    
    // Extract roll and pitch from quaternion orientation
    Eigen::Matrix3d rotation_matrix = orientation.toRotationMatrix();
    double roll = atan2(rotation_matrix(2,1), rotation_matrix(2,2));
    double pitch = asin(-rotation_matrix(2,0));
    
    // Create rotation matrix for roll and pitch only (yaw removed)
    Eigen::Matrix3d roll_pitch_rotation;
    roll_pitch_rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    // Remove gravity using roll and pitch compensation
    // Gravity vector in world frame (0, 0, -9.81)
    Eigen::Vector3d gravity_world(0.0, 0.0, -9.81);
    
    // Remove gravity from raw acceleration to get linear acceleration in vehicle-aligned global frame
    acc = roll_pitch_rotation.transpose() * raw_acc - gravity_world;
    
    // Extract angular velocity data
    gyro.x() = msg.angular_velocity.x;
    gyro.y() = msg.angular_velocity.y;
    gyro.z() = msg.angular_velocity.z;
    return;
}