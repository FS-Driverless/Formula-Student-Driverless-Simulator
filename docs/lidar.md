# Lidar

The lidar sensors are configured in the setting.json.
This is an example lidar:

```
"Lidar1": {
    "SensorType": 6,
    "Enabled": true,
    "X": 0, "Y": 0, "Z": -1,
    "Roll": 0, "Pitch": 0, "Yaw" : 0,
    "NumberOfLasers": 7,
    "PointsPerScan": 2000,
    "RotationsPerSecond": 20,
    "VerticalFOVUpper": 0,
    "VerticalFOVLower": -25,
    "HorizontalFOVStart": 0,
    "HorizontalFOVEnd": 90,
    "DrawDebugPoints": false
}
```

`Lidar1` is the name of the lidar. This value will be used in the ros topic name and coordinate frame.

`X`, `Y` and `Z` are the position of the lidar relative the [vehicle pawn center](vehicle_model.md) of the car in NED frame.

`Roll`,`Pitch` and `Yaw` are rotations in degrees.

`NumberOfLasers` is the - duh - the number of lasers in the lidar.
The lasers are stacked vertically and rotate on the horizontal plane. 
The lasers are distributed equally to cover the specified vertical field of view.
Each laser has a range of 100 meters.

The vertical field of view is specified by choosing the upper (`VerticalFOVUpper`) and lower (`VerticalFOVLower`) limit in degrees. 
The lower limit specifies the vertical angle between the horizontal plane of the lidar and the most bottom laser. 
The upper limit specifies the vertical angle between the horizontal plane of the lidar and most upper laser. 

The horizontal field of view of the lidar is specified with an upper (`HorizontalFOVStart`) and lower (`HorizontalFOVEnd`) limit in degree as well.
The lower limit specifies the counterclockwise angle on a top view (negative yaw) from the direction the lidar is pointing towards.
The upper limit specifies the clockwise angle on a top view (positive yaw) from the direction the lidar is pointing towards. 

`RotationsPerSecond` specifies how fast the lasers spins and how often a pointcloud is captured.
There might be slight variations in the actual lidar frequency vs the configured rotation frequency.

`PointsPerScan` is the number of firings per scan within the field of view.
If all lasers hit, the returned pointcloud will contain this number of points in each pointcloud.

`DrawDebugPoints` enables visualization of the lidar hits inside the unreal engine game.
This is known to impact performance quite a bit. 
it is recommended to to only use this during debugging.