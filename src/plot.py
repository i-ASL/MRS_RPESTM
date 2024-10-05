import rospy
from geometry_msgs.msg import TwistStamped, Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry

class TrajectoryPlotter:
    def __init__(self):
        self.linear_x_values = []
        self.linear_y_values = []
        self.robot2_odom_x = []
        self.robot2_odom_y = []
        self.pub = rospy.Publisher("/pub_esti",Point, queue_size=10)

        # 초기화
        self.robot1_odom_x = []
        self.robot1_odom_y = []
        self._x = 0
        self._y = 0
        rospy.init_node("pl")
        rospy.Subscriber("/robot1/odom", Odometry, self.robot1_odom_callback)
        rospy.Subscriber("/robot2/odom", Odometry, self.robot2_odom_callback)
        rospy.Subscriber("/case_four", TwistStamped, self.callback)

        self.fig, self.ax = plt.subplots()
        self.ani = FuncAnimation(self.fig, self.update, frames=range(100), interval=200)

    def callback(self, data):
        self.linear_x_values.append(data.twist.linear.x-self._x)
        self.linear_y_values.append(data.twist.linear.y-self._y)
        self.pub_data = Point()
        self.pub_data.x =data.twist.linear.x-self._x
        self.pub_data.y =data.twist.linear.y-self._y
        self.pub.publish(self.pub_data)
    def robot1_odom_callback(self, data):
        self._x = data.pose.pose.position.x
        self._y = data.pose.pose.position.y
        self.robot1_odom_x.append(data.pose.pose.position.x)
        self.robot1_odom_y.append(data.pose.pose.position.y)
        # print(self._x)
    def robot2_odom_callback(self, data):
        self.robot2_odom_x.append(data.pose.pose.position.x)
        self.robot2_odom_y.append(data.pose.pose.position.y)

    def update(self, frame):
        self.ax.cla()
        self.ax.scatter(self.robot1_odom_x, self.robot1_odom_y, color='green', s=10, label='Robot 1 Odom')
        self.ax.scatter(self.robot2_odom_x, self.robot2_odom_y, color='blue', s=10, label='Robot 2 Odom')
        self.ax.scatter(self.linear_x_values, self.linear_y_values, color='red', s=10, label='Estimated')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.grid(True)
        plt.legend()

    def run(self):
        plt.show()
        
        rospy.spin()

if __name__ == "__main__":
    plotter = TrajectoryPlotter()
    plotter.run()
