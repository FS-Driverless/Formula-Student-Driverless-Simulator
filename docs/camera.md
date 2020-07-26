# Cameras

To add a camera to your vehicle, add the following json to the "Cameras" map in your `settings.json`:

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

`Camera1` is the name of the camera. This name will be used in ros topics and in coordinate frame.

`X`, `Y` and `Z` are the position of the lidar relative the [vehicle pawn center](vehicle_model.md) of the car in NED frame.

`Roll`,`Pitch` and `Yaw` are rotations in degrees.

`ImageType` describes the type of camera. 
At this moment only rgb and depth cameras are supported.
For rgb camera, set this value to 0 and for depth camera set the value to 2.

* Depth Cameras (aka DepthPerspective) act as follows: each pixel is given a float value in meters corresponding to the smallest distance from the camera to that point. Images published in ros are encoded in `32FC1`
* RGB images are just your normal video camera. Images published in ros are encoded using `bgr8`

`FOV_Degrees` describes [how much the camera sees](https://en.wikipedia.org/wiki/Field_of_view).
The vertical FoV will be automatically calculated using the following formula: `vertical FoV = image height / image width * horizontal FoV`.
