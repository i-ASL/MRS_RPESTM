#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

rospy.init_node('gazebo_cmd_vel')
pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(20)

move1 = Twist()
move1.linear.x = 0.2
move1.angular.z = 0.1

move2 = Twist()
move2.linear.x = 0.4
move2.angular.z = 0.09

while not rospy.is_shutdown():
    pub1.publish(move1)
    pub2.publish(move2)
    rate.sleep()

