#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "csvparser.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_publisher");
    ros::NodeHandle nh;

    CSVParser csvparser("/home/asl/rpestm_ws/src/multiple_turtlebots_estm/src/data.csv");

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/aoa_measurements", 10);

    ros::Rate rate(100);  // 10 Hz

    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        int colSize = csvparser.ncol_;

        std::vector<double> rho;
        std::vector<double> beta;

        for(int i = 0; i < colSize; i++) {
            rho.push_back(csvparser.data_[i][9]);
            beta.push_back(csvparser.data_[i][10]);
        }

        msg.data.insert(msg.data.end(), rho.begin(), rho.end());
        msg.data.insert(msg.data.end(), beta.begin(), beta.end());

        pub.publish(msg);

        for (int i = 0; i < colSize; i++) {
            ROS_INFO("Row %d: rho = %.2f, beta = %.2f", i, rho[i], beta[i]);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}