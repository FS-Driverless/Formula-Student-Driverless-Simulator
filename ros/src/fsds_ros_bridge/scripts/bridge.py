import roslaunch
from os.path import expanduser
import json 

CAMERA_MAX_FRAMERATE = 30

settings = {}

with open(expanduser("~")+'/Formula-Student-Driverless-Simulator/settings.json', 'r') as file:
    settings = json.load(file)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

for cameraname in settings['Vehicles']['FSCar']['Cameras']:
    cameranode = roslaunch.core.Node(
            'fsds_ros_bridge', 'fsds_ros_bridge_camera',
            namespace="fsds/camera", name=cameraname,
            required=True, output='screen',
            args='_camera_name:='+cameraname+' _max_framerate:='+str(CAMERA_MAX_FRAMERATE))
    launch.launch(cameranode)

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.stop()