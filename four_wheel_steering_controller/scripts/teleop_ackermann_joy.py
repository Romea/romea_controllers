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
        self.axis_linear = 1
        self.scale_linear = 1
        self.axis_front_steering = 3
        self.scale_front_steering = pi/10.0

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cmd_four_wheel_steering', FourWheelSteeringDrive, queue_size=10)

        rospy.spin()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons[self.enable_button])
        if data.buttons[self.enable_button] is 1:
            four_wheel_steering_msg = FourWheelSteeringDrive()

            if abs(data.axes[self.axis_linear]) > self.axis_dead_zone:
                four_wheel_steering_msg.speed = data.axes[self.axis_linear]*self.scale_linear        #left up-down stick
            else:
                four_wheel_steering_msg.speed = 0.0

            if abs(data.axes[self.axis_front_steering]) > self.axis_dead_zone:
                four_wheel_steering_msg.steering_angle = data.axes[self.axis_front_steering]*self.scale_front_steering      #right left-right stick
            else:
                four_wheel_steering_msg.steering_angle = 0.0

            self.pub.publish(four_wheel_steering_msg)


if __name__ == '__main__':
    rospy.init_node('teleop_four_wheel_steering_joy', anonymous=False)
    try:
        teleop_four_wheel_steering_joy = TeleopFourWheelSteeringJoy()
    except rospy.ROSInterruptException: pass
