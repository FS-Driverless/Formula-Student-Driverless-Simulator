#include "common/common_utils/StrictMode.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "statistics.h"



typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

// number of seconds to record frames before printing fps
const int FPS_WINDOW = 3;

msr::airlib::CarRpcLibClient* airsim_api;
ros::Publisher image_pub;
ros_bridge::Statistics fps_statistic;

// settings
std::string camera_name = "";
double framerate = 0.0;
std::string host_ip = "localhost";

ros::Time make_ts(uint64_t unreal_ts)
{
   // unreal timestamp is a unix nanosecond timestamp
   // Ros also uses unix timestamps, as long as it is not running in simulated time mode.
   // For now we are not supporting simulated time.
   ros::Time out;
   return out.fromNSec(unreal_ts);
}

std::string logprefix() {
    return "[cambridge " + camera_name + "] ";
}

void doImageUpdate(const ros::TimerEvent&)
{
    // We are using simGetImages instead of simGetImage because the latter does not return image dimention information.
    std::vector<ImageRequest> req;
    req.push_back(ImageRequest(camera_name, ImageType::Scene, false, false));
    std::vector<ImageResponse> img_responses = airsim_api->simGetImages(req, "FSCar");

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0)
        return;

    ImageResponse img_response = img_responses[0];

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();

    img_msg->data = img_response.image_data_uint8;
    img_msg->height = img_response.height;
    img_msg->width = img_response.width;
    img_msg->step = img_response.width * 8; // image_width * num_bytes
    img_msg->encoding = "rgb8";
    img_msg->is_bigendian = 0;
    img_msg->header.stamp = make_ts(img_response.time_stamp);
    img_msg->header.frame_id = "/fsds/camera/"+camera_name;
    
    image_pub.publish(img_msg);
    fps_statistic.addCount();
}

void printFps(const ros::TimerEvent&)
{
    std::cout << logprefix() << "Average FPS: " << (fps_statistic.getCount()/FPS_WINDOW) << std::endl;
    fps_statistic.Reset();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fsds_ros_bridge_camera");
    ros::NodeHandle nh("~");

    // load settings
    nh.param<std::string>("camera_name", camera_name, "");
    nh.param<double>("framerate", framerate, 0.0);
    nh.param<std::string>("host_ip", host_ip, "localhost");

    if(camera_name == "") {
        std::cout << logprefix() << "camera_name unset." << std::endl;
        return 1;
    }
    if(framerate == 0) {
        std::cout << logprefix() << "framerate unset." << std::endl;
        return 1;
    }

    // initialize fps counter
    fps_statistic = ros_bridge::Statistics("fps");

    // ready airsim connection
    msr::airlib::CarRpcLibClient client(host_ip, RpcLibPort, 5);
    airsim_api = &client;

    try {
        airsim_api->confirmConnection();
    } catch (const std::exception &e) {
        std::string msg = e.what();
        std::cout << logprefix() << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
        return 1;
    }

    // ready topic
    image_pub = nh.advertise<sensor_msgs::Image>("/fsds/camera/" + camera_name, 1);

    // start the loop
    ros::Timer imageTimer = nh.createTimer(ros::Duration(1/framerate), doImageUpdate);
    ros::Timer fpsTimer = nh.createTimer(ros::Duration(FPS_WINDOW), printFps);
    ros::spin();
    return 0;
} 
