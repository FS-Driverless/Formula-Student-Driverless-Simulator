#!/usr/bin/env python
import roslaunch
from os.path import expanduser
import json 
import signal
import sys

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

if(len(settings['Vehicles']['FSCar']['Cameras']) == 0):
  # when no camera's are available, ros launch would exit immediatly and thus failing. 
  # And this process failing will take down the parent launchfile as well. 
  # So we just let the process wait for a signal.
  def signal_handler(sig, frame):
    sys.exit(0)
  signal.signal(signal.SIGINT, signal_handler)
  print('No cameras to launch. Camera ros bridge launcher waiting for exit signal')
  signal.pause()

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
    # launch.launch(
    #   roslaunch.core.Node(
    #     'tf', 'static_transform_publisher',
    #     namespace="fsds/camera/"+cameraname, name='tf',
    #     required=True, output='screen',
    #     # see http://wiki.ros.org/tf#static_transform_publisher why this works
    #     args="static_transform_publisher "+str(camsettings['X'])+" "+str(camsettings['Y'])+" "+str(camsettings['Z'])+" "+str(camsettings['Yaw'])+" "+str(camsettings['Pitch'])+" "+str(camsettings['Roll'])+" /fsds/FSCar /fsds/"+cameraname+" 1000"))

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.stop()