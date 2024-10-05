#include "kalman_filter.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>


static constexpr size_t DIM_X{ 3 };
static constexpr size_t DIM_Z{ 2 };


namespace kf
{
    class EKFNode
    {
    public:
        EKFNode()
        {
            ros::NodeHandle nh;
            ekf_pub = nh.advertise<std_msgs::Float32>("ekf_estimate", 10);
            sensor1_sub = nh.subscribe("/measured_range_noise", 10, &EKFNode::callback1, this);
            sensor2_sub = nh.subscribe("/measured_bearing_noise", 10, &EKFNode::callback2, this);
        }

        void callback1(const std_msgs::Float32::ConstPtr& msg){
            float range_measurement = msg->data;
            // call EKF predict

            publishEstimate();
        }

        void callback2(const std_msgs::Float32::ConstPtr& msg){
            float bearing_measurement = msg->data;
            // call EKF predict

            publishEstimate();
        }

        void publishEstimate(){
            std_msgs::Float32 ekf_estimate_msg;
            ekf_estimate_msg.data = ekf.getState();
            ekf_pub.publish(ekf_estimate_msg);
        }

    private:
        KalmanFilter<DIM_X, DIM_Z> ekf;
        ros::Publisher ekf_pub;
        ros::Subscriber sensor1_sub;
        ros::Subscriber sensor2_sub;
    };
}