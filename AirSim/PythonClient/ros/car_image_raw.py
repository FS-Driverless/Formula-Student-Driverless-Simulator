#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

import setup_path 
import airsim
import time
import rospy

# ROS Image message
from sensor_msgs.msg import Image

def airpub():
    pub = rospy.Publisher("airsim/image_raw", Image, queue_size=1)
    rospy.init_node('image_raw', anonymous=True)
    rate = rospy.Rate(15)# connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
         # get camera images from the car
        start = time.time()
        responses = client.simGetImages([
            airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("front_left_custom", airsim.ImageType.Scene, False, False)
            ])
        print(time.time()-start)

        for response in responses:
            img_rgb_string = response.image_data_uint8

        # Populate image message
        msg=Image() 
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frameId"
        msg.encoding = "rgb8"
        msg.height = 480  # resolution should match values in settings.json 
        msg.width = 640
        msg.data = img_rgb_string
        msg.is_bigendian = 0
        msg.step = msg.width * 3

        # log time and size of published image
        # publish image message
        pub.publish(msg)
        # sleep until next cycle
        #rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass
