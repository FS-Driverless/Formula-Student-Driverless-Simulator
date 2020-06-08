#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //Ignore errors inside the rpc package
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
    STRICT_MODE_ON

#include "ros/ros.h"
#include <ros/console.h>
#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include <chrono> 



typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

using namespace std::chrono; 


msr::airlib::CarRpcLibClient* airsim_api;
image_transport::ImageTransport* image_transporter;
image_transport::Publisher* image_pub;

std::string camera_name = "";

ros::Time first_ros_ts;
int64_t first_unreal_ts = -1;

ros::Time make_ts(uint64_t unreal_ts)
{
    if (first_unreal_ts < 0)
    {
        first_unreal_ts = unreal_ts;
        first_ros_ts = ros::Time::now();
    }
    return first_ros_ts + ros::Duration((unreal_ts - first_unreal_ts) / 1e9);
}

auto last = high_resolution_clock::now(); 

std::vector<unsigned char> v(4928400);

void doImageUpdate(const ros::TimerEvent&)
{
    auto now = high_resolution_clock::now(); 
    auto duration = duration_cast<microseconds>(now - last); 
    std::cout << "w" << duration.count() << "\n";
    last = now; 
   

    std::vector<ImageRequest> req;
    req.push_back(ImageRequest(camera_name, ImageType::Scene, false, false));
    std::vector<ImageResponse> img_response = airsim_api->simGetImages(req, "FSCar");

    // if a render request failed for whatever reason, this img will be empty.
    // Attempting to use a make_ts(0) results in ros::Duration runtime error.
    if (img_response.size() == 0 || img_response[0].time_stamp == 0)
        return;

    ImageResponse curr_img_response = img_response[0];

    now = high_resolution_clock::now(); 
    duration = duration_cast<microseconds>(now - last); 
    std::cout << "r" << duration.count() << "\n";
    last = now;


    // todo: publish tf

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();

    img_msg->data = curr_img_response.image_data_uint8;
    img_msg->step = curr_img_response.width * 8; // image_width * num_bytes
    img_msg->header.stamp = make_ts(curr_img_response.time_stamp);
    img_msg->header.frame_id = camera_name+"_optical";
    img_msg->height = curr_img_response.height;
    img_msg->width = curr_img_response.width;
    img_msg->encoding = "rgb8";
    img_msg->is_bigendian = 0;

    image_pub->publish(img_msg);

    now = high_resolution_clock::now(); 
    duration = duration_cast<microseconds>(now - last); 
    std::cout << "p" << duration.count() << "\n";
    last = now;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fsds_ros_bridge_camera", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    image_transporter = new image_transport::ImageTransport(nh);

    nh.param<std::string>("camera_name", camera_name, "front_right_custom");
    msr::airlib::CarRpcLibClient client;
    airsim_api = &client;

    auto p = image_transporter->advertise("/fsds/camera/" + camera_name, 1);
    image_pub = &p;


    try {
        airsim_api->confirmConnection();
    } catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    //60hz
    ros::Timer timer = nh.createTimer(ros::Duration(0.016), doImageUpdate);
    ros::spin();
    return 0;
} 
