# Cameras

You know what camera's are.
There are two camera types:

* RGB camera captures color images, just like any normal normal video camera.
* Depth Cameras (aka DepthPerspective) act as follows: each pixel is given a float value in meters corresponding to the smallest distance from the camera to that point. It captures the world in 3d!

At the moment there is no added noise to the images.

# Add a camera to the car

To add a camera to your vehicle, add the following json to the `Cameras` map in your `settings.json`:

```
"Camera1": {
    "CaptureSettings": [{
        "ImageType": 0,
        "Width": 785,
        "Height": 785,
        "FOV_Degrees": 90
    }],
    "X": 1.0,
    "Y": 0.06,
    "Z": -2.20,
    "Pitch": 0.0,
    "Roll": 0.0,
    "Yaw": 180
}
```

`Camera1` is the name of the camera. 
This name will be used to reference the camera when collecing images.

`X`, `Y` and `Z` are the position of the lidar relative the [vehicle pawn center](vehicle_model.md) of the car in ENU frame.

`Roll`,`Pitch` and `Yaw` are rotations in degrees.

`ImageType` describes the type of camera. 
At this moment only rgb and depth cameras are supported.
For rgb camera, set this value to 0 and for depth camera set the value to 2.

`FOV_Degrees` describes [how much the camera sees](https://en.wikipedia.org/wiki/Field_of_view).
The vertical FoV will be automatically calculated using the following formula: `vertical FoV = image height / image width * horizontal FoV`.

## Python

```python
"""
Args:
    requests (list[ImageRequest]): Images required
    vehicle_name (str, optional): Name of vehicle associated with the camera

Returns:
    list[ImageResponse]
"""
[image] = client.simGetImages([fsds.ImageRequest()], vehicle_name = 'FSCar')

# For color images:
[image] = client.simGetImages([fsds.ImageRequest(camera_name = 'Camera1', image_type = fsds.ImageType.Scene, pixels_as_float = False, compress = False)], vehicle_name = 'FSCar')

# For depth images images:
[image] = client.simGetImages([fsds.ImageRequest(camera_name = 'Camera1', image_type = fsds.ImageType.DepthPerspective, pixels_as_float = True, compress = False)], vehicle_name = 'FSCar')
```

The API simGetImages can accept request for multiple image types from any cameras in single call. 
You can specify if image is png compressed, RGB uncompressed or float array. 
For png compressed images, you get binary string literal.
For float array you get Python list of float64. 

See [this file](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/tree/master/python/examples/camera_color_png.py) with an example on how get acolor image and write it to a file.
[Do you want to hep with writing more examples?](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/240)

## ROS

When using the ROS bridge, images are published on the `/fsds/camera/CAMERA_NAME` topic, where `CAMERA_NAME` will be replaced by the name defined in the `settings.json`.
Every camera specified in the `settings.json` file get's its own topic. 

The message type is [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html).
The encoding of the image data is `bgra8` for color images and `32FC1` for depth images.

The ROS bridge regularly publishes static [transforms](http://wiki.ros.org/tf) between the `fsds/FSCar` frame and every camera.
Naming of the camera frames is `fsds/CAMERA_NAME`.
For example, the position and orientation of a camera named `Test` will become available in the frame `/fsds/Test`.