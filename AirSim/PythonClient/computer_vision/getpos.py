# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

client = airsim.VehicleClient()
client.confirmConnection()

print("x={}, y={}, z={}".format(pose.position.x_val, pose.position.y_val, pose.position.z_val))

print("pitch={}, roll={}, yaw={}".format(angles[0], angles[1], angles[2]))

pose.position.x_val = pose.position.x_val + 1
client.simSetVehiclePose(pose, True)