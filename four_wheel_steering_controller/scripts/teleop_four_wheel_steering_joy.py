#!/usr/bin/env python
import rospy
from math import pi
from four_wheel_steering_msgs.msg import FourWheelSteeringDrive
from sensor_msgs.msg import Joy

class TeleopFourWheelSteeringJoy():
    def __init__(self):
        self.enable_button = 4
        self.is_enable_button = False
        self.axis_dead_zone = 0.05
        self.axis_linear_forward = 5
        self.axis_linear_reverse = 2
        self.scale_linear = 1
        self.axis_front_steering = 0
        self.axis_rear_steering = 3
        self.scale_steering = pi/10.0

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cmd_four_wheel_steering', FourWheelSteeringDrive, queue_size=10)

        rospy.spin()

    def callback(self, data):
        four_wheel_steering_msg = FourWheelSteeringDrive()
        rospy.loginfo(rospy.get_caller_id() + " axes" + str(data.axes))
        speed = (-data.axes[self.axis_linear_forward] + data.axes[self.axis_linear_reverse])/2.0
        rospy.loginfo(rospy.get_caller_id() + " speed %s", speed)
        if abs(speed) > self.axis_dead_zone:
            four_wheel_steering_msg.speed = speed*self.scale_linear

            if abs(data.axes[self.axis_front_steering]) > self.axis_dead_zone:
                four_wheel_steering_msg.front_steering_angle = data.axes[self.axis_front_steering]*self.scale_steering      #left left-right stick
            else:
                four_wheel_steering_msg.front_steering_angle = 0.0

            if abs(data.axes[self.axis_rear_steering]) > self.axis_dead_zone:
                four_wheel_steering_msg.rear_steering_angle = data.axes[self.axis_rear_steering]*self.scale_steering      #right left-right stick
            else:
                four_wheel_steering_msg.rear_steering_angle = 0.0

        self.pub.publish(four_wheel_steering_msg)


if __name__ == '__main__':
    rospy.init_node('teleop_four_wheel_steering_joy', anonymous=False)
    try:
        teleop_four_wheel_steering_joy = TeleopFourWheelSteeringJoy()
    except rospy.ROSInterruptException: pass
