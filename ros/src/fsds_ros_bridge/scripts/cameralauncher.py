#!/usr/bin/env python
import roslaunch
from os.path import expanduser
import json 

CAMERA_FRAMERATE = 30
AIRSIM_HOSTIP = "localhost"

settings = {}

def args(argsmap):
  out = ""
  for k in argsmap:
    out += ' _' + str(k) + ":=" + str(argsmap[k])
  return out

with open(expanduser("~")+'/Formula-Student-Driverless-Simulator/settings.json', 'r') as file:
    settings = json.load(file)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

for cameraname in settings['Vehicles']['FSCar']['Cameras']:
    camsettings = settings['Vehicles']['FSCar']['Cameras'][cameraname]
    launch.launch(
      roslaunch.core.Node(
        'fsds_ros_bridge', 'fsds_ros_bridge_camera',
        namespace="fsds/camera", name=cameraname,
        required=True, output='screen',
        args=args({
          'camera_name': cameraname,
          'framerate': CAMERA_FRAMERATE,
          'host_ip': AIRSIM_HOSTIP
        })))
    launch.launch(
      roslaunch.core.Node(
        'tf', 'static_transform_publisher',
        namespace="fsds/camera/"+cameraname, name='tf',
        required=True, output='screen',
        # see http://wiki.ros.org/tf#static_transform_publisher why this works
        args="static_transform_publisher "+str(camsettings['X'])+" "+str(camsettings['Y'])+" "+str(camsettings['Z'])+" "+str(camsettings['Yaw'])+" "+str(camsettings['Pitch'])+" "+str(camsettings['Roll'])+" /fsds/FSCar /fsds/"+cameraname+" 1000"))

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.stop()