#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def pose_callback(pose_stamped):
    quaternion = pose_stamped.pose.orientation
    quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion_list)
    

    rospy.loginfo("Received pose:")
    # rospy.loginfo("Roll: {:.2f} degrees".format(roll * 180.0 / 3.14159))
    # rospy.loginfo("Pitch: {:.2f} degrees".format(pitch * 180.0 / 3.14159))
    rospy.loginfo("Yaw: {:.2f} degrees".format(yaw * 180.0 / 3.14159))

def main():
    rospy.init_node('pose_to_euler_converter', anonymous=True)
    rospy.Subscriber('/Robot_0/ground_truth', PoseStamped, pose_callback)
    rospy.spin()

if __name__=='__main__':
    main()