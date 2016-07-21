#!/usr/bin/env python
import rospy
from math import pi
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

class TeleopAckermannJoy():
    def __init__(self):
        self.axis_dead_zone = 0.05
        self.axis_linear = 1
        self.scale_linear = 1
        self.axis_front_steering = 3
        self.scale_front_steering = pi/10.0

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cmd_ackermann', AckermannDrive, queue_size=10)
        rate = rospy.Rate(10.0)
        self.linear_speed = 0.0
        self.front_steering = 0.0

        while rospy.is_shutdown() is False:
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = self.linear_speed
            ackermann_msg.steering_angle = self.front_steering
            self.pub.publish(ackermann_msg)
            rate.sleep()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
        if abs(data.axes[self.axis_linear]) > self.axis_dead_zone:
            self.linear_speed = data.axes[self.axis_linear]*self.scale_linear        #left up-down stick
        else:
            self.linear_speed = 0.0

        if abs(data.axes[self.axis_front_steering]) > self.axis_dead_zone:
            self.front_steering = data.axes[self.axis_front_steering]*self.scale_front_steering      #right left-right stick
        else:
            self.front_steering = 0.0


if __name__ == '__main__':
    rospy.init_node('teleop_ackermann_joy', anonymous=False)
    try:
        teleop_ackermann_joy = TeleopAckermannJoy()
    except rospy.ROSInterruptException: pass
