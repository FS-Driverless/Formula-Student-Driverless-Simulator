#!/usr/bin/env python
import rospy, math
from fsds_ros_bridge.msg import ControlCommand

# the number of seconds to do a full steering cycle (neutral -> full right -> full left -> neutroal)
STEERING_PERIOD = 5

# hz at what the car setpoints are published
SETPOINT_FREQUENCY = 5.0

# The throtle. This is just a static value.
THROTLE = 0.5

def publishloop():
    controllpublisher = rospy.Publisher('/fsds_ros_bridge/FSCar/control_command', ControlCommand, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    time = 0
    while not rospy.is_shutdown():
        cc = ControlCommand()
        cc.header.stamp = rospy.Time.now()
        cc.throttle = THROTLE
        cc.steering = math.sin(time * (math.pi / STEERING_PERIOD))
        cc.brake = 0

        controllpublisher.publish(cc)
        time += 1.0 / SETPOINT_FREQUENCY
        rate.sleep()
        

def listeners():
   pass

if __name__ == '__main__':
    rospy.init_node('example_sinewave')
    listeners()
    publishloop()