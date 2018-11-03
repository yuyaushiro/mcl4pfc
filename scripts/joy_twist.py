#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTwist(object):
    def __init__(self):
        hz = 5.0
        rate = rospy.Rate(hz)
        self._max_vel_x = 0.15
        self._max_vel_theta = math.radians(45)
        self._sub_joy =\
            rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        self._pub_cmd_vel =\
            rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._twist = Twist()
        while not rospy.is_shutdown():
            self._pub_cmd_vel.publish(self._twist)
            rate.sleep()

    def joy_callback(self, joy_msg):
        self._twist.linear.x = joy_msg.axes[1] * self._max_vel_x
        self._twist.angular.z = joy_msg.axes[3] * self._max_vel_theta

if __name__ == '__main__':
    rospy.init_node('joy_twist')
    try:
        joy_twist = JoyTwist()
    except rospy.ROSInterruptException:
        pass
