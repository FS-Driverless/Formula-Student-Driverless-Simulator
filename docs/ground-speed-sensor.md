# Ground Speed Sensor (GSS)

The ground speed sensor is modeled around the Kistler ground speed (like the Kistler Correvit SFII).

Velocity information is captured in the global world frame in ENU frame.

At this moment no extra noise is added to the sensordata since the kistler 250hz data averaged into the 100hz is so close to ground truth that adding noise would be unrealistic.

## Ros
When using the ros bridge, ground speed sensordata will be published on `/fsds/gss` with the `geometry_msgs/TwistStamped` message type.

Appart from the header fields, only `x`, `y` and `z` of the `twist.linear` are populated. 

```
header:
  seq: 5747
  stamp:
    secs: 1595325426
    nsecs: 617730500
  frame_id: "fsds/FSCar"
twist:
  linear:
    x: 4.80838251114
    y: -0.0
    z: -0.0214105024934
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
```