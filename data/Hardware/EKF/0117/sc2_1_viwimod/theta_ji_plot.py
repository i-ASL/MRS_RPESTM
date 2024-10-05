import matplotlib.pyplot as plt
import numpy as np
def thetaplot():
    with open('theta_ji_gt.txt', 'r') as file:
        gt_theta_ji = file.readlines()

    with open('bag_theta_ji.txt', 'r') as file:
        bag_theta_ji = file.readlines()

    with open('rosbag_time.txt', 'r') as file:
        rostime = file.readlines()

    gt_theta = []
    bag_theta = []

    gt_lin_vel = []
    gt_ang_vel = []

    bag_v_j = []
    bag_w_j = []

    rosgbag_time = []



    for line in gt_theta_ji:
        gt_theta.append(float(line.strip())) 

    for line in bag_theta_ji:
        bag_theta.append(float(line.strip())*60.58244544)

    for line in rostime:
        rosgbag_time.append(float(line.strip()))



    color = 'tab:red'
    plt.plot(rosgbag_time, bag_theta, marker='s', linestyle='-', color=color, label='estimation of theta_ji')

    color = 'tab:blue'
    plt.plot(rosgbag_time, gt_theta, marker='s', linestyle='-', color=color, label='GT of theta_ji')

    # color = 'tab:green'
    # plt.plot(gt_robot1_x, gt_robot1_y, marker='s', linestyle='-', color=color, label='gt_r1')

    plt.title('Estimation')
    plt.xlabel('Time')
    plt.ylabel('rad/s')
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__=="__main__":
    thetaplot()