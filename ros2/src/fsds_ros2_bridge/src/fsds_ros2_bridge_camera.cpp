#include "common/common_utils/StrictMode.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "statistics.h"
#include "rpc/rpc_error.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>

using dseconds = std::chrono::duration<double>;

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

// number of seconds to record frames before printing fps
const int FPS_WINDOW = 3;

msr::airlib::CarRpcLibClient *airsim_api;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
ros_bridge::Statistics fps_statistic;

// settings
std::string camera_name = "";
double framerate = 0.0;
std::string host_ip = "localhost";
bool depthcamera = false;

rclcpp::Time make_ts(uint64_t unreal_ts)
{
    // unreal timestamp is a unix nanosecond timestamp just like ros.
    // We can do direct translation as long as ros is not running in simulated time mode.
    return rclcpp::Time(unreal_ts);
}

std::vector<ImageResponse> getImage(ImageRequest req) {
     // We are using simGetImages instead of simGetImage because the latter does not return image dimention information.
    std::vector<ImageRequest> reqs;
    reqs.push_back(req);

    std::vector<ImageResponse> img_responses;
    try
    {
        img_responses = airsim_api->simGetImages(reqs, "FSCar");
    }
    catch (rpc::rpc_error &e)
    {
        std::cout << "error" << std::endl;
        std::string msg = e.what();
        std::cout << "Exception raised by the API while getting image:" << std::endl
                  << msg << std::endl;
    }
    return img_responses;
}

void doImageUpdate()
{
    auto img_responses = getImage(ImageRequest(camera_name, ImageType::Scene, false, false));

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0)
        return;

    ImageResponse img_response = img_responses[0];

    sensor_msgs::msg::Image::SharedPtr img_msg = std::make_shared<sensor_msgs::msg::Image>();

    img_msg->data = img_response.image_data_uint8;
    img_msg->height = img_response.height;
    img_msg->width = img_response.width;
    img_msg->step = img_response.width * 3; // image_width * num_bytes
    img_msg->encoding = "bgr8";
    img_msg->is_bigendian = 0;
    img_msg->header.stamp = make_ts(img_response.time_stamp);
    img_msg->header.frame_id = "/fsds/" + camera_name;

    image_pub->publish(*img_msg);

    sensor_msgs::msg::CameraInfo::SharedPtr info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info_msg->header.stamp = make_ts(img_response.time_stamp);
    info_msg->header.frame_id = "/fsds/" + camera_name;
    info_msg->width = img_response.width;
    info_msg->height = img_response.height;
    info_msg->distortion_model = "plumb_bob";
    info_msg->d = {0, 0, 0, 0, 0};

    double fx = static_cast<double>(img_response.width) / 2;
    double fy = fx;
    double cx = fx;
    double cy = static_cast<double>(img_response.height) / 2;

    info_msg->k = {fx, 0, cx,
                   0, fy, cy,
                   0, 0, 1.0};

    info_msg->r = {1, 0, 0,
                   0, 1, 0,
                   0, 0, 1};

    fps_statistic.addCount();
}

cv::Mat manual_decode_depth(const ImageResponse &img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

float roundUp(float numToRound, float multiple)
{
    assert(multiple);
    return ((numToRound + multiple - 1) / multiple) * multiple;
}

cv::Mat noisify_depthimage(cv::Mat in)
{
    cv::Mat out = in.clone();

    // Blur
    cv::Mat kernel = cv::Mat::ones(7, 7, CV_32F) / (float)(7 * 7);
    cv::filter2D(in, out, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // Decrease depth resolution
    for (int row = 0; row < in.rows; row++)
    {
        for (int col = 0; col < in.cols; col++)
        {
            float roundtarget = ceil(std::min(std::max(out.at<float>(row, col), 1.0f), 10.0f));
            out.at<float>(row, col) = roundUp(out.at<float>(row, col), roundtarget);
        }
    }

    return out;
}

void doDepthImageUpdate()
{
    auto img_responses = getImage(ImageRequest(camera_name, ImageType::DepthPerspective, true, false));

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0)
        return;

    ImageResponse img_response = img_responses[0];

    cv::Mat depth_img = noisify_depthimage(manual_decode_depth(img_response));
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_img).toImageMsg();
    img_msg->header.stamp = make_ts(img_response.time_stamp);
    img_msg->header.frame_id = "/fsds/" + camera_name;

    image_pub->publish(*img_msg);
    fps_statistic.addCount();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("fsds_ros2_bridge_camera");

    // load settings
    camera_name = nh->declare_parameter<std::string>("camera_name", "");
    framerate = nh->declare_parameter<double>("framerate", 0.0);
    host_ip = nh->declare_parameter<std::string>("host_ip", "localhost");
    depthcamera = nh->declare_parameter<bool>("depthcamera", false);

    if(camera_name == "") {
        RCLCPP_FATAL(nh->get_logger(), "camera_name unset.");
        return 1;
    }
    if(framerate == 0) {
        RCLCPP_FATAL(nh->get_logger(), "framerate unset.");
        return 1;
    }

    // initialize fps counter
    fps_statistic = ros_bridge::Statistics("fps");

    // ready airsim connection
    msr::airlib::CarRpcLibClient client(host_ip, RpcLibPort, 5);
    airsim_api = &client;

    double timeout_sec = nh->declare_parameter<double>("timeout", 10.0);

    try {
        std::cout << "Waiting for connection - " << std::endl;
        airsim_api->confirmConnection(timeout_sec);
        std::cout << "Connected to the simulator!" << std::endl;
    } catch (const std::exception &e) {
        std::string msg = e.what();
        RCLCPP_ERROR(nh->get_logger(), "Exception raised by the API, something went wrong: %s\n", msg.c_str());
        return 1;
    }

    // ready topic
    image_pub = nh->create_publisher<sensor_msgs::msg::Image>("/fsds/" + camera_name + "/image_color", 1);
    info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>("/fsds/" + camera_name + "/camera_info", 1);

    // start the loop
    rclcpp::TimerBase::SharedPtr imageTimer = nh->create_wall_timer(dseconds { 1/framerate }, depthcamera ? &doDepthImageUpdate : &doImageUpdate);
    rclcpp::TimerBase::SharedPtr fpsTimer = nh->create_wall_timer(dseconds { FPS_WINDOW }, [&nh](){
        RCLCPP_DEBUG(nh->get_logger(), "Average FPS: %d\n", fps_statistic.getCount()/FPS_WINDOW);
        fps_statistic.Reset();
    });
    rclcpp::spin(nh);
    return 0;
}
