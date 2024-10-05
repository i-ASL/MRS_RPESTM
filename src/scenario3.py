#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math as m
import time

time_2 = 50
radius_2 = 0.8

rospy.init_node('gazebo_cmd_vel')
pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(20)

move1 = Twist()
move2 = Twist()
move1.linear.x = 0
move1.angular.z = 0
move2.linear.x = 2*m.pi*radius_2/time_2
move2.angular.z = 2*m.pi/time_2

#+0.05*m.cos(rad*180/m.pi)
while not rospy.is_shutdown():
    pub1.publish(move1)
    pub2.publish(move2)
    rate.sleep()

