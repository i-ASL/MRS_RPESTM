import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Header
import time

# Lists to store angular.x and angular.y values for plotting
time_values = []
angular_x_values = []
angular_y_values = []

def callback(data):
    # Get the current time
    timestamp = rospy.Time.now().to_sec()

    # Append the timestamp and angular values to the lists
    time_values.append(timestamp)
    angular_x_values.append(data.angular.x)
    angular_y_values.append(data.angular.y)

def update(frame):
    # Clear the current axis to update the plot
    plt.cla()

    # Plot angular.x and angular.y over time with different colors
    plt.plot(time_values, angular_x_values, label='Angular X', color='blue')
    plt.plot(time_values, angular_y_values, label='Angular Y', color='green')

    # Set labels and title
    plt.xlabel('Time')
    plt.ylabel('Angular Value')
    plt.title('/tt Topic Data')

    # Enable legend for better visualization
    plt.legend()

    # Enable grid for better visualization
    plt.grid(True)

def main():
    # Initialize ROS node
    rospy.init_node("plotter_node", anonymous=True)

    # Subscribe to the "/tt" topic
    rospy.Subscriber("/tt", Twist, callback)

    # Set up the plot and animation
    fig, ax = plt.subplots()
    ani = FuncAnimation(fig, update, frames=None, interval=200)

    # Show the plot
    plt.show()

if __name__ == "__main__":
    main()
