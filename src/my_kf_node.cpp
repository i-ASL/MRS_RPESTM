#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;

using float32_t = float;
float prev_m_vecX_2 = 0;
template <size_t ROW, size_t COL>
using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

template<size_t ROW>
using Vector = Eigen::Matrix<float32_t, ROW, 1>;

const float32_t v_i = 0.1;
const float32_t w_i = 0.0;
const float32_t v_j = 0.1;
const float32_t w_j = 0.0;


// Relative Pose Estimation with measurements of Range and Bearing without Communication between Robots
class EKFRangePlusBearingWithoutCommunication
{
private:
    // Prediction
    ros::NodeHandle n_;
    Vector<5> m_vecX;
    Matrix<5, 5> m_matP{Matrix<5, 5>::Zero()};
    Matrix<5, 5> m_matQ{Matrix<5, 5>::Zero()};
    Matrix<5, 5> m_jacobian_matF{Matrix<5, 5>::Zero()};
    ros::Publisher _pub_;
    // Correction
    Vector<2> m_vecZ; // measurement data
    Vector<2> m_vech; // measurement data
    Matrix<2, 2> m_matR{Matrix<2, 2>::Zero()};
    Matrix<2, 5> m_jacobian_matH{Matrix<2, 5>::Zero()};

public:
    EKFRangePlusBearingWithoutCommunication()
    {
        _pub_ = n_.advertise<geometry_msgs::TwistStamped>("/for_plot", 1);
        // Variables Initialization

        m_vecX[0] = 1.0;
        m_vecX[1] = 1.0;
        m_vecX[2] = 1.5;
        m_vecX[3] = 0.1;
        m_vecX[4] = 0.1;

        m_matP(0,0) = 0.0;
        m_matP(1,1) = 0.0;
        m_matP(2,2) = 0.0;
        m_matP(3,3) = 0.0;
        m_matP(4,4) = 0.0;

        // process noise (prediction)
        m_matQ(0,0) = 0.0001; // x
        m_matQ(1,1) = 0.0001; // y
        m_matQ(2,2) = 0.0001; // theta
        m_matQ(3,3) = 0.0001; // vel
        m_matQ(4,4) = 0.0001; // omega

        // measurement noise (correction)
        m_matR(0,0) = 0.0001; // range
        m_matR(1,1) = 0.001; // bearing
    }

    Vector<5>& getVecX()
    {
        return m_vecX;
    }

    Matrix<5, 5>& getMatP()
    {
        return m_matP;
    }
    Vector<2>& getvecZ()
    {
        return m_vecZ;
    }

    // Motion Model
    void motionModelJacobian(Vector<5>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -vec[3] * sin(vec[2]), cos(vec[2]), 0,
                          -w_i, 0, vec[3] * cos(vec[2]), sin(vec[2]), 0,
                           0, 0, 0, 0, 1,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0;
        m_jacobian_matF = Matrix<5, 5>::Identity() + delta_t *m_jacobian_matF;
    }

    void motionModel(Vector<5>& vec, float32_t delta_t)
    {
        Vector<5> tmp_vec;
                    //Vj
        tmp_vec[0] = vec[3]*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = vec[3]*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = vec[4] - w_i;
        tmp_vec[3] = 0;
        tmp_vec[4] = 0;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    // Prediction Model
    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    // Measurement Model
    void measurementModel(Vector<5>& vec)
    {
        m_vech[0] = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
        m_vech[1] = atan2(vec[1], vec[0]);
        if (m_vech[1] > M_PI)
            m_vech[1] =  M_PI;
        else if(m_vech[1] < -M_PI)
            m_vech[1] =  -M_PI;
    }

    void measurementModelJacobian(Vector<5>& vec)
    {
        m_jacobian_matH << vec[0] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), vec[1] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), 0, 0, 0,
                          -vec[1] / (vec[0] * vec[0] + vec[1] * vec[1]), vec[0] / (vec[0] * vec[0] + vec[1] * vec[1]), 0, 0, 0;
    }

    void correction()
    {
        measurementModel(m_vecX);
        measurementModelJacobian(m_vecX);


        // residual(innovation)
        Vector<2> residual;

        residual[0] = m_vecZ[0] - m_vech[0];
        residual[1] = m_vecZ[1] - m_vech[1];


        // residual(innovation) covariance
        Matrix<2, 2> residual_cov;
        residual_cov = m_jacobian_matH * m_matP * m_jacobian_matH.transpose() + m_matR;

        // Kalman Gain
        Matrix<5, 2> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        std::cout << "vecZ is : " << m_vecZ[1]*180/M_PI << ", " << "vecH is : " << m_vech[1]*180/M_PI <<", "<< "m_vexX[2] is : " << m_vecX[2]*180/M_PI<< endl;
        m_matP = (Matrix<5,5>::Identity() - Kk * m_jacobian_matH) * m_matP;
        
        
        geometry_msgs::TwistStamped msg2;
        msg2.twist.linear.x = m_vecZ[1]*180/M_PI;
        msg2.twist.linear.y = m_vech[1]*180/M_PI;
        msg2.twist.linear.z = m_vecX[2]*180/M_PI;
        // if(msg2.twist.linear.z>360)
        //     msg2.twist.linear.z -=360;
        msg2.twist.angular.x = 0;
        msg2.twist.angular.y = 0;
        msg2.twist.angular.z = 0;
        _pub_.publish(msg2);
        // prev_m_vecX_2 = m_vecX[2];
        // if(abs(prev_m_vecX_2 - m_vecX[2]) <0.1)
        //     m_vecX[2] =  prev_m_vecX_2;
    }

    void print()
    {
        std::cout << m_vecX[0] <<", " <<  m_vecX[1] << ", "<< m_vecX[2] * 180 / M_PI << std::endl;
    }

    void print2()
    {
        std::cout << "The z value is " << std::endl << m_vecZ << std::endl;
    }
};


class EKFNode
{
public:
    EKFNode()
    {
        pub_range_and_bearing_without_comm_ = n_.advertise<geometry_msgs::TwistStamped>("case_four", 1);
        
        sub_ = n_.subscribe("/aoa_measurements", 1, &EKFNode::callback, this);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        // ROS_INFO("Hi");
        static ros::Time previousTime;

        ros::Time currentTime = msg->header.stamp;

        if (previousTime.isZero())
        {
            previousTime = currentTime;
            return;
        }

        ros::Duration duration = currentTime - previousTime;
        float32_t delta_t = duration.toSec();
        if (delta_t != 0)
        {
           ekf_range_plus_bearing_without_comm.getvecZ() << msg->point.x, msg->point.y * M_PI / 180;
           ekf_range_plus_bearing_without_comm.prediction(delta_t);
           ekf_range_plus_bearing_without_comm.correction();
           geometry_msgs::TwistStamped msg;
           msg.header.stamp = currentTime;

           msg.twist.linear.x = ekf_range_plus_bearing_without_comm.getVecX()[0];
           msg.twist.linear.y = ekf_range_plus_bearing_without_comm.getVecX()[1];
           msg.twist.linear.z = ekf_range_plus_bearing_without_comm.getVecX()[2]*M_PI/180;
            // msg.twist.linear.z = m_vecX[0]
           msg.twist.angular.x = ekf_range_plus_bearing_without_comm.getVecX()[3];
           msg.twist.angular.y = ekf_range_plus_bearing_without_comm.getVecX()[4]* 180 / M_PI;
           msg.twist.angular.z = 0.0;
           
        //    if (msg.twist.linear.z < M_PI)
        //        msg.twist.linear.z += 2 * M_PI;

           
           pub_range_and_bearing_without_comm_.publish(msg);
           

           previousTime = currentTime;
        }
    }
private:
    ros::NodeHandle n_;

    ros::Publisher pub_range_and_bearing_without_comm_;

    ros::Subscriber sub_;


    EKFRangePlusBearingWithoutCommunication ekf_range_plus_bearing_without_comm;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "EkfNode");
    ROS_INFO("Init!");
    EKFNode object;
    ros::spin();

    return 0;

}