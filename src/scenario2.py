#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math as m
import time
time_1 = 100
radius_1 = 1.5
rad =0
time_2 = 90
radius_2 = 2
accel = 0.01

rospy.init_node('gazebo_cmd_vel')
pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(20)

move1 = Twist()
# move1.linear.x = 0.2
# move1.angular.z = 0.1
move1.linear.x = 2 * m.pi * radius_1 / time_1 ## 0.094
move1.angular.z = 2*m.pi/time_1 ## 0.063

move2 = Twist()
# move2.linear.x = 0.4
# move2.angular.z = 0.09
move2.linear.x = 2 * m.pi * radius_2 / time_2 ## 0.140
move2.angular.z = 2*m.pi/time_2 ## 0.070
#+0.05*m.cos(rad*180/m.pi)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    time_elapsed = (current_time - start_time).to_sec()
    # Update speed with constant accel
    move2.linear.x += accel * time_elapsed

    pub1.publish(move1)
    pub2.publish(move2)
    rate.sleep()

