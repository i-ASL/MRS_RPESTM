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

template <size_t ROW, size_t COL>
using Matrix = Eigen::Matrix<float32_t, ROW, COL>;

template<size_t ROW>
using Vector = Eigen::Matrix<float32_t, ROW, 1>;

const float32_t v_i = 0.2;
const float32_t w_i = 0.1;
const float32_t v_j = 0.4;
const float32_t w_j = 0.09;


class EKF
{
private:
    // Prediction
    Vector<3> m_vecX;
    Matrix<3, 3> m_matP{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_matQ{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_jacobian_matF{Matrix<3, 3>::Zero()};

    // Correction
    Vector<3> m_vecZ; // measurement
    Vector<3> m_vech; // measurement data
    Matrix<3, 3> m_matR{    Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_jacobian_matH{Matrix<3, 3>::Zero()};

public:
    EKF()
    {
        m_vecX[0] = 1.0;
        m_vecX[1] = 1.0;
        m_vecX[2] = 0.0;

        m_matP(0,0) = 0.0;
        m_matP(1,1) = 0.0;
        m_matP(2,2) = 0.0;

        m_matQ(0,0) = 0.003;
        m_matQ(1,1) = 0.01;
        m_matQ(2,2) = 0.003;

        m_matR(0,0) = 0.1;
        m_matR(1,1) = 0.1;
        m_matR(2,2) = 0.1;
    }

    Vector<3>& getVecX()
    {
        return m_vecX;
    }

    Matrix<3, 3>& getMatP()
    {
        return m_matP;
    }
    Vector<3>& getvecZ()
    {
        return m_vecZ;
    }

//    void test(Vector<3>& vec)
//    {
//        m_vecX[0] = vec[0];
//        m_vecX[1] = vec[1];
//        m_vecX[2] = vec[2];
//    }

    void motionModelJacobian(Vector<3>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -v_j * sin(vec[2]),
                          -w_i, 0, v_j * cos(vec[2]),
                           0, 0, 0;
        m_jacobian_matF = Matrix<3, 3>::Identity() + delta_t * m_jacobian_matF;
    }

    void motionModel(Vector<3>& vec, float32_t delta_t)
    {
        Vector<3> tmp_vec;

        tmp_vec[0] = v_j*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = v_j*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = w_j - w_i;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    void measurementModel(Vector<3>& vec)
    {
        m_vech[0] = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
        m_vech[1] = atan2(vec[1], vec[0]);
        if (m_vech[1] < 0)
            m_vech[1] += 2 * M_PI;
//        std::cout << m_vech[1] << std::endl;
        m_vech[2] = vec[2];
    }

    void measurementModelJacobian(Vector<3>& vec)
    {
        m_jacobian_matH << vec[0] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), vec[1] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), 0,
                          -vec[1] / (vec[0] * vec[0] + vec[1] * vec[1]), vec[0] / (vec[0] * vec[0] + vec[1] * vec[1]), 0,
                              0, 0, 1;
    }

    void correction()
    {
        measurementModel(m_vecX);
        measurementModelJacobian(m_vecX);

        // residual(innovation)
        Vector<3> residual;
        residual[0] = m_vecZ[0] - m_vech[0];
//        std::cout << m_vech[0] << std::endl;
        residual[1] = m_vecZ[1] - m_vech[1];
        residual[2] = m_vecZ[2] - m_vech[2];

        // residual(innovation) covariance
        Matrix<3, 3> residual_cov;
        residual_cov = m_jacobian_matH * m_matP * m_jacobian_matH.transpose() + m_matR;

        // Kalman Gain
        Matrix<3, 3> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        m_matP = (Matrix<3,3>::Identity() - Kk * m_jacobian_matH) * m_matP;
    }

    void print()
    {
//        std::cout << m_vecX[0] <<", " <<  m_vecX[1] << ", "<< m_vecX[2] * 180 / M_PI << std::endl;
//        std::cout << "The state cov is " << std::endl << m_matP << std::endl;
//        std::cout << sqrt(m_vecX[0]*m_vecX[0] + m_vecX[1]*m_vecX[1]) << ", " << atan2(m_vecX[1], m_vecX[0]) * 180 / M_PI << ", " << m_vecX[2] * 180 / M_PI << std::endl;
    }

    void print2()
    {
        std::cout << "The z value is " << std::endl << m_vecZ << std::endl;
    }
};

class EKFRangePlusBearing
{
private:
    // Prediction
    Vector<3> m_vecX;
    Matrix<3, 3> m_matP{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_matQ{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_jacobian_matF{Matrix<3, 3>::Zero()};

    // Correction
    Vector<2> m_vecZ; // measurement
    Vector<2> m_vech; // measurement data
    Matrix<2, 2> m_matR{    Matrix<2, 2>::Zero()};
    Matrix<2, 3> m_jacobian_matH{Matrix<2, 3>::Zero()};

public:
    EKFRangePlusBearing()
    {
        m_vecX[0] = 1.0;
        m_vecX[1] = 1.0;
        m_vecX[2] = 0.0;

        m_matP(0,0) = 0.0;
        m_matP(1,1) = 0.0;
        m_matP(2,2) = 0.0;

        m_matQ(0,0) = 0.003;
        m_matQ(1,1) = 0.01;
        m_matQ(2,2) = 0.003;

        m_matR(0,0) = 0.1;
        m_matR(1,1) = 0.1;
    }

    Vector<3>& getVecX()
    {
        return m_vecX;
    }

    Matrix<3, 3>& getMatP()
    {
        return m_matP;
    }
    Vector<2>& getvecZ()
    {
        return m_vecZ;
    }

    void motionModelJacobian(Vector<3>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -v_j * sin(vec[2]),
                          -w_i, 0, v_j * cos(vec[2]),
                           0, 0, 0;
        m_jacobian_matF = Matrix<3, 3>::Identity() + delta_t * m_jacobian_matF;
    }

    void motionModel(Vector<3>& vec, float32_t delta_t)
    {
        Vector<3> tmp_vec;

        tmp_vec[0] = v_j*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = v_j*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = w_j - w_i;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    void measurementModel(Vector<3>& vec)
    {
        m_vech[0] = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
        m_vech[1] = atan2(vec[1], vec[0]);
        if (m_vech[1] < 0)
            m_vech[1] += 2 * M_PI;
    }

    void measurementModelJacobian(Vector<3>& vec)
    {
        m_jacobian_matH << vec[0] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), vec[1] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), 0,
                          -vec[1] / (vec[0] * vec[0] + vec[1] * vec[1]), vec[0] / (vec[0] * vec[0] + vec[1] * vec[1]), 0;
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
        Matrix<3, 2> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        m_matP = (Matrix<3,3>::Identity() - Kk * m_jacobian_matH) * m_matP;
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

class EKFRangeOnly
{
private:
    // Prediction
    Vector<3> m_vecX;
    Matrix<3, 3> m_matP{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_matQ{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_jacobian_matF{Matrix<3, 3>::Zero()};

    // Correction
    Vector<1> m_vecZ; // measurement
    Vector<1> m_vech; // measurement data
    Matrix<1, 1> m_matR{    Matrix<1, 1>::Zero()};
    Matrix<1, 3> m_jacobian_matH{Matrix<1, 3>::Zero()};

public:
    EKFRangeOnly()
    {
        m_vecX[0] = 2.0;
        m_vecX[1] = -1.0;
        m_vecX[2] = 1.5;

        m_matP(0,0) = 0.1;
        m_matP(1,1) = 0.1;
        m_matP(2,2) = 0.1;

        m_matQ(0,0) = 0.0001;
        m_matQ(1,1) = 0.0001;
        m_matQ(2,2) = 0.001;

        m_matR(0,0) = 0.001;
    }

    Vector<3>& getVecX()
    {
        return m_vecX;
    }

    Matrix<3, 3>& getMatP()
    {
        return m_matP;
    }

    Vector<1>& getvecZ()
    {
        return m_vecZ;
    }

    void motionModelJacobian(Vector<3>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -v_j * sin(vec[2]),
                          -w_i, 0, v_j * cos(vec[2]),
                           0, 0, 0;
        m_jacobian_matF = Matrix<3, 3>::Identity() + delta_t * m_jacobian_matF;
    }

    void motionModel(Vector<3>& vec, float32_t delta_t)
    {
        Vector<3> tmp_vec;

        tmp_vec[0] = v_j*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = v_j*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = w_j - w_i;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    void measurementModel(Vector<3>& vec)
    {
        m_vech[0] = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
    }

    void measurementModelJacobian(Vector<3>& vec)
    {
        m_jacobian_matH << vec[0] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), vec[1] / (sqrt(vec[0] * vec[0] + vec[1] * vec[1])), 0;
    }

    void correction()
    {
        measurementModel(m_vecX);
        measurementModelJacobian(m_vecX);

        // residual(innovation)
        Vector<1> residual;
        residual[0] = m_vecZ[0] - m_vech[0];

        // residual(innovation) covariance
        Matrix<1, 1> residual_cov;
        residual_cov = m_jacobian_matH * m_matP * m_jacobian_matH.transpose() + m_matR;

        // Kalman Gain
        Matrix<3, 1> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        m_matP = (Matrix<3,3>::Identity() - Kk * m_jacobian_matH) * m_matP;
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

class EKFBearingOnly
{
private:
    // Prediction
    Vector<3> m_vecX;
    Matrix<3, 3> m_matP{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_matQ{Matrix<3, 3>::Zero()};
    Matrix<3, 3> m_jacobian_matF{Matrix<3, 3>::Zero()};

    // Correction
    Vector<1> m_vecZ; // measurement
    Vector<1> m_vech; // measurement data
    Matrix<1, 1> m_matR{Matrix<1, 1>::Zero()};
    Matrix<1, 3> m_jacobian_matH{Matrix<1, 3>::Zero()};

public:
    EKFBearingOnly()
    {
        m_vecX[0] = 1.0;
        m_vecX[1] = 1.0;
        m_vecX[2] = 0.0;

        m_matP(0,0) = 0.0;
        m_matP(1,1) = 0.0;
        m_matP(2,2) = 0.0;

        m_matQ(0,0) = 0.001;
        m_matQ(1,1) = 0.001;
        m_matQ(2,2) = 0.001;

        m_matR(0,0) = 0.01;
    }

    Vector<3>& getVecX()
    {
        return m_vecX;
    }

    Matrix<3, 3>& getMatP()
    {
        return m_matP;
    }
    Vector<1>& getvecZ()
    {
        return m_vecZ;
    }

    void motionModelJacobian(Vector<3>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -v_j * sin(vec[2]),
                          -w_i, 0, v_j * cos(vec[2]),
                           0, 0, 0;
        m_jacobian_matF = Matrix<3, 3>::Identity() + delta_t * m_jacobian_matF;
    }

    void motionModel(Vector<3>& vec, float32_t delta_t)
    {
        Vector<3> tmp_vec;

        tmp_vec[0] = v_j*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = v_j*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = w_j - w_i;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    void measurementModel(Vector<3>& vec)
    {
        m_vech[0] = atan2(vec[1], vec[0]);
        if (m_vech[0] < 0)
            m_vech[0] += 2 * M_PI;
    }

    void measurementModelJacobian(Vector<3>& vec)
    {
        m_jacobian_matH << -vec[1] / (vec[0] * vec[0] + vec[1] * vec[1]), vec[0] / (vec[0] * vec[0] + vec[1] * vec[1]), 0;
    }

    void correction()
    {
        measurementModel(m_vecX);
        measurementModelJacobian(m_vecX);

        // residual(innovation)
        Vector<1> residual;
        residual[0] = m_vecZ[0] - m_vech[0];

        // residual(innovation) covariance
        Matrix<1, 1> residual_cov;
        residual_cov = m_jacobian_matH * m_matP * m_jacobian_matH.transpose() + m_matR;

        // Kalman Gain
        Matrix<3, 1> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        m_matP = (Matrix<3,3>::Identity() - Kk * m_jacobian_matH) * m_matP;
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

class EKFRangePlusBearingWithoutCommunication
{
private:
    // Prediction
    Vector<5> m_vecX;
    Matrix<5, 5> m_matP{Matrix<5, 5>::Zero()};
    Matrix<5, 5> m_matQ{Matrix<5, 5>::Zero()};
    Matrix<5, 5> m_jacobian_matF{Matrix<5, 5>::Zero()};

    // Correction
    Vector<2> m_vecZ; // measurement
    Vector<2> m_vech; // measurement data
    Matrix<2, 2> m_matR{Matrix<2, 2>::Zero()};
    Matrix<2, 5> m_jacobian_matH{Matrix<2, 5>::Zero()};

public:
    EKFRangePlusBearingWithoutCommunication()
    {
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

        m_matQ(0,0) = 0.01;
        m_matQ(1,1) = 0.01;
        m_matQ(2,2) = 0.01;
        m_matQ(3,3) = 0.001;
        m_matQ(4,4) = 0.001;

        m_matR(0,0) = 0.001;
        m_matR(1,1) = 0.001;
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

    void motionModelJacobian(Vector<5>& vec, float32_t delta_t)
    {
        m_jacobian_matF << 0, w_i, -vec[3] * sin(vec[2]), cos(vec[2]), 0,
                          -w_i, 0, vec[3] * cos(vec[2]), sin(vec[2]), 0,
                           0, 0, 0, 0, 1,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0;
        m_jacobian_matF = Matrix<5, 5>::Identity() + delta_t * m_jacobian_matF;
    }

    void motionModel(Vector<5>& vec, float32_t delta_t)
    {
        Vector<5> tmp_vec;

        tmp_vec[0] = vec[3]*cos(vec[2]) + w_i*vec[1] - v_i;
        tmp_vec[1] = vec[3]*sin(vec[2]) - w_i*vec[0];
        tmp_vec[2] = vec[4] - w_i;
        tmp_vec[3] = 0;
        tmp_vec[4] = 0;

        m_vecX = m_vecX + delta_t * tmp_vec;
    }

    void prediction(float32_t delta_t)
    {
        motionModelJacobian(m_vecX, delta_t);
        motionModel(m_vecX, delta_t);
        m_matP = m_jacobian_matF * m_matP * m_jacobian_matF.transpose() + m_matQ;
    }

    void measurementModel(Vector<5>& vec)
    {
        m_vech[0] = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
        m_vech[1] = atan2(vec[1], vec[0]);
        if (m_vech[1] < 0)
            m_vech[1] += 2 * M_PI;
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

//        std::cout << "vecZ is : " << m_vecZ[1] << ", " << "vecH is : " << m_vech[1] << endl;

        // residual(innovation) covariance
        Matrix<2, 2> residual_cov;
        residual_cov = m_jacobian_matH * m_matP * m_jacobian_matH.transpose() + m_matR;

        // Kalman Gain
        Matrix<5, 2> Kk = m_matP * m_jacobian_matH.transpose() * residual_cov.inverse();

        // update
        m_vecX = m_vecX + Kk * residual;
        m_matP = (Matrix<5,5>::Identity() - Kk * m_jacobian_matH) * m_matP;
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
        pub_all_ = n_.advertise<geometry_msgs::PointStamped>("case_zero", 1);
        pub_range_only_ = n_.advertise<geometry_msgs::PointStamped>("case_one", 1);
        pub_bearing_only_ = n_.advertise<geometry_msgs::PointStamped>("case_two", 1);
        pub_range_and_bearing_ = n_.advertise<geometry_msgs::PointStamped>("case_three", 1);
        pub_range_and_bearing_without_comm_ = n_.advertise<geometry_msgs::TwistStamped>("case_four", 1);

        sub_ = n_.subscribe("/noisy_measurements", 1, &EKFNode::callback, this);
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
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
            ekf.getvecZ() << msg->point.x, msg->point.y * M_PI / 180, msg->point.z * M_PI / 180;
            ekf.prediction(delta_t);
//            cout << "ekf\n" << " x: " << ekf.getVecX()[0]
//                 << " y: " << ekf.getVecX()[1]
//                 << " z: " << ekf.getVecX()[2] << endl;
            ekf.correction();
//            ekf.print();
           geometry_msgs::PointStamped msg0;
           msg0.header.stamp = currentTime;
//           float32_t tmp_x0 = ekf.getVecX()[0];
//           float32_t tmp_y0 = ekf.getVecX()[1];
//           float32_t tmp_z0 = ekf.getVecX()[2];
//           msg0.point.x = sqrt(tmp_x0 * tmp_x0 + tmp_y0 * tmp_y0);
//           msg0.point.y = atan2(tmp_y0, tmp_x0);
//           if (msg0.point.y < 0)
//               msg0.point.y += 2 * M_PI;
//           msg0.point.y *= 180 / M_PI;
//           msg0.point.z = tmp_z0;
//           if (msg0.point.z < 0)
//               msg0.point.z += 2 * M_PI;
           msg0.point.z *= 180 / M_PI;
           msg0.point.x = ekf.getVecX()[0];
           msg0.point.y = ekf.getVecX()[1];
           msg0.point.z = ekf.getVecX()[2];

           pub_all_.publish(msg0);


           ekf_range_plus_bearing.getvecZ() << msg->point.x, msg->point.y * M_PI / 180;
           ekf_range_plus_bearing.prediction(delta_t);
//           cout << "ekf_range_plus_bearing\n" << " x: " << ekf_range_plus_bearing.getVecX()[0]
//                << " y: " << ekf_range_plus_bearing.getVecX()[1]
//                << " z: " << ekf_range_plus_bearing.getVecX()[2] << endl;
           ekf_range_plus_bearing.correction();
           geometry_msgs::PointStamped msg1;
           msg1.header.stamp = currentTime;
//           float32_t tmp_x1 = ekf_range_plus_bearing.getVecX()[0];
//           float32_t tmp_y1 = ekf_range_plus_bearing.getVecX()[1];
//           float32_t tmp_z1 = ekf_range_plus_bearing.getVecX()[2];
//           msg1.point.x = sqrt(tmp_x1 * tmp_x1 + tmp_y1 * tmp_y1);
//           msg1.point.y = atan2(tmp_y1, tmp_x1);
//           if (msg1.point.y < 0)
//               msg1.point.y += 2 * M_PI;
//           msg1.point.y *= 180 / M_PI;
//           msg1.point.z = tmp_z1;
//           if (msg1.point.z < 0)
//               msg1.point.z += 2 * M_PI;
//           if (msg1.point.z > 2 * M_PI)
//               msg1.point.z -= 2 * M_PI;
//           msg1.point.z *= 180 / M_PI;
           msg1.point.x = ekf_range_plus_bearing.getVecX()[0];
           msg1.point.y = ekf_range_plus_bearing.getVecX()[1];
           msg1.point.z = ekf_range_plus_bearing.getVecX()[2];
           if (msg1.point.z < 0)
               msg1.point.z += 2 * M_PI;

           pub_range_and_bearing_.publish(msg1);


           ekf_range_only.getvecZ() << msg->point.x;
           ekf_range_only.prediction(delta_t);
//           cout << "ekf_range_only\n" << " x: " << ekf_range_only.getVecX()[0]
//                << " y: " << ekf_range_only.getVecX()[1]
//                << " z: " << ekf_range_only.getVecX()[2] << endl;
           ekf_range_only.correction();
           geometry_msgs::PointStamped msg2;
           msg2.header.stamp = currentTime;
//           float32_t tmp_x2 = ekf_range_only.getVecX()[0];
//           float32_t tmp_y2 = ekf_range_only.getVecX()[1];
//           float32_t tmp_z2 = ekf_range_only.getVecX()[2];
//           msg2.point.x = sqrt(tmp_x2 * tmp_x2 + tmp_y2 * tmp_y2);
//           msg2.point.y = atan2(tmp_y2, tmp_x2);
//           if (msg2.point.y < 0)
//               msg2.point.y += 2 * M_PI;
//           msg2.point.y *= 180 / M_PI;
//           msg2.point.z = tmp_z2;
//           if (msg2.point.z < 0)
//               msg2.point.z += 2 * M_PI;
//           msg2.point.z *= 180 / M_PI;
           msg2.point.x = ekf_range_only.getVecX()[0];
           msg2.point.y = ekf_range_only.getVecX()[1];
           msg2.point.z = ekf_range_only.getVecX()[2];
           if (msg2.point.z < 0)
               msg2.point.z += 2 * M_PI;

           pub_range_only_.publish(msg2);


           ekf_bearing_only.getvecZ() << msg->point.y * M_PI / 180;
           ekf_bearing_only.prediction(delta_t);
           ekf_bearing_only.correction();
//           ekf_bearing_only.print();
           geometry_msgs::PointStamped msg3;
           msg3.header.stamp = currentTime;
//           float32_t tmp_x3 = ekf_bearing_only.getVecX()[0];
//           float32_t tmp_y3 = ekf_bearing_only.getVecX()[1];
//           float32_t tmp_z3 = ekf_bearing_only.getVecX()[2];
//           msg3.point.x = sqrt(tmp_x3 * tmp_x3 + tmp_y3 * tmp_y3);
//           msg3.point.y = atan2(tmp_y3, tmp_x3);
//           if (msg3.point.y < 0)
//               msg3.point.y += 2 * M_PI;
//           msg3.point.y *= 180 / M_PI;
//           msg3.point.z = tmp_z3;
//           cout << "Before : " <<  msg3.point.z << endl;
//           if ( -2 * M_PI < msg3.point.z < 0)
//               msg3.point.z +=2 * M_PI;
//           if ( -4 * M_PI < msg3.point.z < -2 * M_PI)
//               msg3.point.z += 4 * M_PI;
//           if (msg3.point.z > 2 * M_PI)
//               msg3.point.z -= 2 * M_PI;
//           cout << "After : " << msg3.point.z << endl;
//           msg3.point.z *= 180 / M_PI;
           msg3.point.x = ekf_bearing_only.getVecX()[0];
           msg3.point.y = ekf_bearing_only.getVecX()[1];
           msg3.point.z = ekf_bearing_only.getVecX()[2];
           if (msg3.point.z < M_PI)
               msg3.point.z += 2 * M_PI;

           pub_bearing_only_.publish(msg3);


           ekf_range_plus_bearing_without_comm.getvecZ() << msg->point.x, msg->point.y * M_PI / 180;
           ekf_range_plus_bearing_without_comm.prediction(delta_t);
//           cout << "ekf_range_plus_bearing\n" << " x: " << ekf_range_plus_bearing.getVecX()[0]
//                << " y: " << ekf_range_plus_bearing.getVecX()[1]
//                << " z: " << ekf_range_plus_bearing.getVecX()[2] << endl;
           ekf_range_plus_bearing_without_comm.correction();
           geometry_msgs::TwistStamped msg4;
           msg4.header.stamp = currentTime;
//           float32_t tmp_x4 = ekf_range_plus_bearing_without_comm.getVecX()[0];
//           float32_t tmp_y4 = ekf_range_plus_bearing_without_comm.getVecX()[1];
//           float32_t tmp_z4 = ekf_range_plus_bearing_without_comm.getVecX()[2];
//           msg1.point.x = sqrt(tmp_x1 * tmp_x1 + tmp_y1 * tmp_y1);
//           msg1.point.y = atan2(tmp_y1, tmp_x1);
//           if (msg1.point.y < 0)
//               msg1.point.y += 2 * M_PI;
//           msg1.point.y *= 180 / M_PI;
//           msg1.point.z = tmp_z1;
//           if (msg1.point.z < 0)
//               msg1.point.z += 2 * M_PI;
//           if (msg1.point.z > 2 * M_PI)
//               msg1.point.z -= 2 * M_PI;
//           msg1.point.z *= 180 / M_PI;
           msg4.twist.linear.x = ekf_range_plus_bearing_without_comm.getVecX()[0];
           msg4.twist.linear.y = ekf_range_plus_bearing_without_comm.getVecX()[1];
           msg4.twist.linear.z = ekf_range_plus_bearing_without_comm.getVecX()[2];
           msg4.twist.angular.x = ekf_range_plus_bearing_without_comm.getVecX()[3];
           msg4.twist.angular.y = ekf_range_plus_bearing_without_comm.getVecX()[4];
           msg4.twist.angular.z = 0.0;

           if (msg4.twist.linear.z < M_PI)
               msg4.twist.linear.z += 2 * M_PI;

           pub_range_and_bearing_without_comm_.publish(msg4);


           previousTime = currentTime;
        }
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_all_;
    ros::Publisher pub_range_only_;
    ros::Publisher pub_bearing_only_;
    ros::Publisher pub_range_and_bearing_;
    ros::Publisher pub_range_and_bearing_without_comm_;
    ros::Subscriber sub_;

    EKF ekf;
    EKFRangeOnly ekf_range_only;
    EKFBearingOnly ekf_bearing_only;
    EKFRangePlusBearing ekf_range_plus_bearing;
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
