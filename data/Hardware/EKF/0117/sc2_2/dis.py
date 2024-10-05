import matplotlib.pyplot as plt
def angularplot():
    with open('dis.txt', 'r') as file:
        lin_vel_r2 = file.readlines()


    with open('rosbag_time.txt', 'r') as file:
        rostime = file.readlines()

    gt_lin_vel = []

    rosgbag_time = []



    for line in lin_vel_r2:
        gt_lin_vel.append(float(line.strip())) 

    for line in rostime:
        rosgbag_time.append(float(line.strip()))



    color = 'tab:red'
    plt.plot(rosgbag_time, gt_lin_vel, marker='s', linestyle='-', color=color, label='estimation of angular velocity')
    plt.title('Estimation')
    plt.show()


if __name__ =="__main__":
    angularplot()
