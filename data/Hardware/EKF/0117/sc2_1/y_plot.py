import matplotlib.pyplot as plt
import numpy as np
def yplot():
    with open('rosbag_x.txt', 'r') as file:
        rosbag_x = file.readlines()

    with open('rosbag_y.txt', 'r') as file:
        rosbag_y = file.readlines()

    with open('rosbag_time.txt', 'r') as file:
        rostime = file.readlines()

    with open('theta_i.txt', 'r') as file:
        theta_i = file.readlines()

    # with open('rosbag_time_plus.txt', 'r') as file:
    #     rostime_plus = file.readlines()

    with open('sc1_robot1_gt_X.txt', 'r') as file:
        robot1_gt_x = file.readlines()

    with open('sc1_robot2_gt_X.txt', 'r') as file:
        robot2_gt_x = file.readlines()

    with open('sc1_robot1_gt_Y.txt', 'r') as file:
        robot1_gt_y = file.readlines()

    with open('sc1_robot2_gt_Y.txt', 'r') as file:
        robot2_gt_y = file.readlines()


    esti_x = []
    esti_y = []
    thetaI = []
    _esti_x = []
    _esti_y = []

    rosgbag_time = []
    rosgbag_time_plus = []

    gt_robot1_x = []
    gt_robot1_y = []

    gt_robot2_x = []
    gt_robot2_y = []

    for line in rostime:
        rosgbag_time.append(float(line.strip()))

    # for line in rostime_plus:
    #     rosgbag_time_plus.append(float(line.strip()))
    for line in theta_i:
        thetaI.append(float(line.strip()))
    for line in rosbag_x:
        esti_x.append(float(line.strip()))
        _esti_x.append(0)
    for line in rosbag_y:
        esti_y.append(float(line.strip()))
        _esti_y.append(0)

    for line in robot1_gt_x:
        gt_robot1_x.append(float(line.strip()))

    for line in robot2_gt_x:
        gt_robot2_x.append(float(line.strip()))

    for line in robot1_gt_y:
        gt_robot1_y.append(float(line.strip()))

    for line in robot2_gt_y:
        gt_robot2_y.append(float(line.strip()))
    min_length = min(len(gt_robot1_x), len(esti_x), len(esti_y))

    for i in range(min_length):
        _esti_x[i] = np.cos(thetaI[i]*np.pi/180+np.pi/2) * esti_x[i] - np.sin(thetaI[i]*np.pi/180+np.pi/2) * esti_y[i]  + gt_robot1_x[i] 
        _esti_y[i] = np.sin(thetaI[i]*np.pi/180+np.pi/2) * esti_x[i] + np.cos(thetaI[i]*np.pi/180+np.pi/2) * esti_y[i] + gt_robot1_y[i]  
        
    color = 'tab:red'
    plt.plot(rosgbag_time, _esti_y, marker='s', linestyle='-', color=color, label='Estimation of Robot2 Y')

    color = 'tab:blue'
    plt.plot(rosgbag_time, gt_robot2_y, marker='s', linestyle='-', color=color, label='GT of Robot2 Y')

    # color = 'tab:green'
    # plt.plot(gt_robot1_x, gt_robot1_y, marker='s', linestyle='-', color=color, label='gt_r1')

    plt.title('EKF')
    plt.xlabel('Time')
    plt.ylabel('Estimated Y')
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ =="__main__":
    yplot()
