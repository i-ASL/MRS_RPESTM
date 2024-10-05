#include <iostream>
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "kalman_filter.h"

static constexpr size_t DIM_X{ 2 };
static constexpr size_t DIM_Z{ 2 };

static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter;



kf::Vector<DIM_Z> convertCartesian2Polar(const kf::Vector<DIM_X>& cartesian)
{
    const kf::Vector<DIM_Z> polar{
        std::sqrt(cartesian[0]*cartesian[0] + 
                  cartesian[1]*cartesian[1]),
        std::atan2(cartesian[1], cartesian[0])
    };
    return polar;
}

kf::Matrix<DIM_Z, DIM_Z> calculateJacobianMatrix(const kf::Vector<DIM_X>& vecX)
{
    const kf::float32_t valX2PlusY2{ (vecX[0]*vecX[0]) + (vecX[1]*vecX[1]) };
    const kf::float32_t valSqrtX2PlusY2{ std::sqrt(valX2PlusY2) };

    kf::Matrix<DIM_Z, DIM_X> matHj;
    matHj << 
        (vecX[0] / valSqrtX2PlusY2), (vecX[1] / valSqrtX2PlusY2),
        (-vecX[1] / valX2PlusY2), (vecX[0] / valX2PlusY2);
    
    return matHj;
}

void executeCorrectionStep()
{
    kalmanfilter.vecX() << 10.0F, 5.0F;
    kalmanfilter.matP() << 0.3F, 0.0F, 0.0F, 0.3F;

    const kf::Vector<DIM_X> measPosCart{ 10.4F, 5.2F };
    const kf::Vector<DIM_Z> vecZ{ convertCartesian2Polar(measPosCart) };

    kf::Matrix<DIM_Z, DIM_Z> matR;
    matR << 0.1F, 0.0F, 0.0F, 0.0008F;

    kf::Matrix<DIM_Z, DIM_X> matHj{ calculateJacobianMatrix(kalmanfilter.vecX()) };

    kalmanfilter.correctEkf(convertCartesian2Polar, vecZ, matR, matHj);

    std::cout << "\ncorrected state vector = \n" << kalmanfilter.vecX() << "\n";
    std::cout << "\ncorrected state covariance = \n" << kalmanfilter.matP() << "\n";
}

class EKFNode{
public:
    EKFNode()
    {
        ros::NodeHandle nh;
        ekf_pub = nh.advertise<std_msgs::Float32>("ekf_estimate", 10);
        sensor1_sub = nh.subscribe("sensor1_data", 10, &EKFNode::sensor1Callback, this);
        sensor2_sub = nh.subscribe("sensor2_data", 10, &EKFNode::sensor2Callback, this);
    }

    void sensor1Callback(const std_msgs::Float32::ConstPtr& msg)
    {}

    void sensor2Callback(const std_msgs::Float32::ConstPtr& msg)
    {}

private:
    ros::Publisher ekf_pub;
    ros::Subscriber sensor1_sub;
    ros::Subscriber sensor2_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    EKFNode ekf_node;
    ros::spin();

    return 0;
}

