#!/usr/bin/env python
import rospy, math
from fs_msgs.msg import ControlCommand
from fs_msgs.msg import FinishedSignal

# the number of seconds to do a full steering cycle (neutral -> full right -> full left -> neutroal)
STEERING_PERIOD = 5

# hz at what the car setpoints are published
SETPOINT_FREQUENCY = 5.0

# The throtle. This is just a static value.
THROTLE = 0.5

rospy.init_node('example_sinewave')
controllpublisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
finishedpublisher = rospy.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)

time = 0

def publish(x):
    global time
    global controllpublisher
    cc = ControlCommand()
    cc.header.stamp = rospy.Time.now()
    cc.throttle = THROTLE
    cc.steering = math.sin(time * (math.pi / STEERING_PERIOD))
    cc.brake = 0
    controllpublisher.publish(cc)

    time += 1.0 / SETPOINT_FREQUENCY

def finish(x):
    finishedpublisher.publish(FinishedSignal())

rospy.Timer(rospy.Duration(0.1), publish)
rospy.Timer(rospy.Duration(30), finish)

rospy.spin()
