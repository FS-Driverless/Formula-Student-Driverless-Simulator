# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint
import os
import time

pp = pprint.PrettyPrinter(indent=4)

client = airsim.VehicleClient()
client.confirmConnection()

airsim.wait_key('Press any key to set camera-0 gimble to 15-degree pitch')
client.simSetCameraOrientation("0", airsim.to_quaternion(0.261799, 0, 0)); #radians

airsim.wait_key('Press any key to get camera parameters')
for camera_name in range(5):
    camera_info = client.simGetCameraInfo(str(camera_name))
    print("CameraInfo %d: %s" % (camera_name, pp.pprint(camera_info)))

airsim.wait_key('Press any key to get images')
for x in range(3): # do few times
    z = x * -20 - 5 # some random number
    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(z, z, z), airsim.to_quaternion(x / 3.0, 0, x / 3.0)), True)

    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),
        airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True),
        airsim.ImageRequest("2", airsim.ImageType.Segmentation),
        airsim.ImageRequest("3", airsim.ImageType.Scene),
        airsim.ImageRequest("4", airsim.ImageType.DisparityNormalized),
        airsim.ImageRequest("4", airsim.ImageType.SurfaceNormals)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
            airsim.write_pfm(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), airsim.get_pfm_array(response))
        else:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
            airsim.write_file(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)

    pp.pprint(pose)

    time.sleep(3)

# currently reset() doesn't work in CV mode. Below is the workaround
client.simSetPose(airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0)), True)