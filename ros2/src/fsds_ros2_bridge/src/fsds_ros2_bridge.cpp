#include "rclcpp/rclcpp.hpp"
#include "airsim_ros_wrapper.h"
// #include <ros/spinner.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fsds_ros2_bridge"); 

    std::string host_ip = node->declare_parameter<std::string>("host_ip", "localhost");
    double timeout_sec = node->declare_parameter<double>("timeout", 10.0);

    AirsimROSWrapper airsim_ros_wrapper(node, host_ip, timeout_sec);

    // if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_)
    // {
    //     airsim_ros_wrapper.lidar_async_spinner_.start();
    // }

    rclcpp::spin(node);

    return 0;
} 
