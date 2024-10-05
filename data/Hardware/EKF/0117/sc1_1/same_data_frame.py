import rospy
import rosbag

data_x = []
data_y = []
data_z = []
cnt = 0
csv_data_frame = 1200
rosbag_data_frame = 1467
def callback(msg):
    global cnt
    cnt +=1
    # print(cnt)
    if cnt%rosbag_data_frame/(rosbag_data_frame-csv_data_frame) <= 1:
        pass
    else:
        data_x.append(msg.twist.linear.z)
        data_y.append(msg.twist.angular.x)
        data_z.append(msg.twist.angular.y)


def make_txt_file_for_plot():
    with open("0117_anglular.txt", "w") as file:
        for x, y, z  in zip(data_x, data_y, data_z):
            file.write(f"{x}\t {y}\t {z}\n")

if __name__ =="__main__":
    rospy.init_node("same_frame")
    bag_path = "/home/kim/mul_ws/uam_bag/0117/sc1_1.bag"  
    topic_name = "/case_four"
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics = [topic_name]):
            callback(msg)

    make_txt_file_for_plot()