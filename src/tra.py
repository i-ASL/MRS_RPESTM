import rospy
from geometry_msgs.msg import TwistStamped, Twist
import math as m

rospy.init_node("tr")
pub = rospy.Publisher("/tt", Twist, queue_size=10)
tr = Twist()

def cb(data):
    global tr
    # TwistStamped 메시지에서 Twist 부분 추출
    twist_data = data.twist

    # 선형 요소 업데이트
    tr.linear.x = twist_data.linear.x
    tr.linear.y = twist_data.linear.y
    tr.linear.z = twist_data.linear.z

    # 각 요소 업데이트 (라디안으로 변환)
    tr.angular.x = twist_data.angular.x 
    tr.angular.y = twist_data.angular.y * m.pi / 180.0
    tr.angular.z = twist_data.angular.z

rospy.Subscriber("/case_four", TwistStamped, callback=cb)

# ROS rate 생성하여 게시 빈도 제어
rate = rospy.Rate(10)  # 10 Hz, 필요에 따라 조절

while not rospy.is_shutdown():
    pub.publish(tr)
    rate.sleep()

rospy.spin()
