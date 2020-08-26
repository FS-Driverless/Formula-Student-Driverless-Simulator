"""
This example shows how to retrieve an color image and store it as a png file
Before running this you must add the camera to your vehicle.
Add the following to your settings.json file in the Cameras section:

"examplecam": {
    "CaptureSettings": [
    {
        "ImageType": 0,
        "Width": 785,
        "Height": 785,
        "FOV_Degrees": 90
    }
    ],
    "X": 1.0,
    "Y": 0.06,
    "Z": -1.20,
    "Pitch": 0.0,
    "Roll": 0.0,
    "Yaw": 0
},

"""

import sys
import os

## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import fsds

# connect to the simulator 
client = fsds.FSDSClient()

# Check network connection, exit if not connected
client.confirmConnection()

# Get the image
[image] = client.simGetImages([fsds.ImageRequest(camera_name = 'examplecam', image_type = fsds.ImageType.Scene, pixels_as_float = False, compress = True)], vehicle_name = 'FSCar')

print("Image width: ", image.width)
print("Image height: ", image.height)

# write to png 
fsds.write_file(os.path.normpath('example.png'), image.image_data_uint8)