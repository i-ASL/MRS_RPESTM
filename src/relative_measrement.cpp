#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <random>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <iostream>

using namespace std;

class NoisyMeasurements
{
private:
    ros::NodeHandle n_;
    ros::Rate rate_;

    ros::Publisher pub1_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;

    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Subscriber sub4_;

    geometry_msgs::PoseStamped pose1;
    geometry_msgs::PoseStamped pose2;
    geometry_msgs::PoseStamped pose3;
    geometry_msgs::PoseStamped pose4;

    geometry_msgs::PointStamped current_odom;

    geometry_msgs::TwistStamped twist_1;
    geometry_msgs::TwistStamped twist_2;

    ros::Time previous_time;

    double mean = 0.0;
    double stdDev = 0.05;

public:
    NoisyMeasurements() : rate_(20.0)
    {
        // measurements publisher
        pub1_ = n_.advertise<geometry_msgs::PointStamped>("/noisy_measurements", 1);
        pub2_ = n_.advertise<geometry_msgs::PointStamped>("/ground_truth", 1);

        // odometry publisher
        pub3_ = n_.advertise<geometry_msgs::PointStamped>("/odometry", 1);

        // 섭스크라이브할 토픽 선언
        sub1_ = n_.subscribe("/Robot_0/ground_truth", 1, &NoisyMeasurements::callback1, this);
        sub2_ = n_.subscribe("/Robot_1/ground_truth", 1, &NoisyMeasurements::callback2, this);
        sub3_ = n_.subscribe("/robot1/cmd_vel", 1, &NoisyMeasurements::callback3, this);
        sub4_ = n_.subscribe("/robot2/cmd_vel", 1, &NoisyMeasurements::callback4, this);

    }

    void callback1(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose1 = *msg;
//        calculateMeasurementAndPublish();
//        trueMeasurementAndPublish();
    }

    void callback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose2 = *msg;
        calculateMeasurementAndPublish();
        trueMeasurementAndPublish();
    }

    void callback3(const geometry_msgs::Twist::ConstPtr& msg)
    {
        twist_1.header.stamp = ros::Time::now();
        twist_1.twist = *msg;
    }

    void callback4(const geometry_msgs::Twist::ConstPtr& msg)
    {
        twist_2.header.stamp = ros::Time::now();
        twist_2.twist = *msg;
        odometryPublish();
    }


    void calculateMeasurementAndPublish()
    {
        if (!pose1.header.stamp.isZero() && !pose2.header.stamp.isZero())
        {
            double distance =  sqrt((pose1.pose.position.x - pose2.pose.position.x) * (pose1.pose.position.x - pose2.pose.position.x)
                            +(pose1.pose.position.y - pose2.pose.position.y) * (pose1.pose.position.y - pose2.pose.position.y));
            addGaussianNoise(distance, mean, stdDev);


            tf2::Quaternion quat1, quat2;
            tf2::fromMsg(pose1.pose.orientation, quat1);
            tf2::fromMsg(pose2.pose.orientation, quat2);
            double yaw1 = tf2::getYaw(quat1);
            if (yaw1 < 0)
                yaw1 += 2 * M_PI;

            double yaw2 = tf2::getYaw(quat2);
            if (yaw2 < 0)
                yaw2 += 2 * M_PI;

            double yaw_diff = yaw1 - yaw2;

            if (yaw_diff < 0)
                yaw_diff += 2 * M_PI;
//            std::cout << yaw_diff << std::endl;

            yaw_diff = 2*M_PI - yaw_diff;
//            std::cout << yaw_diff * 180 / M_PI << std::endl;
//            addGaussianNoise(yaw_diff, mean, stdDev);
//            std::cout << yaw_diff << std::endl;
            yaw_diff *= 180 / M_PI;


            yaw1 *= 180 / M_PI;
//            std::cout << yaw1 << std::endl;
            double x_diff = pose2.pose.position.x - pose1.pose.position.x;
            double y_diff = pose2.pose.position.y - pose1.pose.position.y;
            double bearing = atan2(y_diff , x_diff);
//            std::cout << bearing << std::endl;
            addGaussianNoise(bearing, mean, stdDev);
            if (bearing < 0)
                bearing += 2 *M_PI;
            bearing = bearing * (180 / M_PI) - yaw1;
            if (bearing < 0)
                bearing += 360;
//            addGaussianNoise(bearing, mean, stdDev);

            geometry_msgs::PointStamped measurements;
            measurements.header = pose1.header;
            measurements.point.x = distance;
            measurements.point.y = bearing;
            measurements.point.z = yaw_diff;

            pub1_.publish(measurements);
        }
    }

    void trueMeasurementAndPublish()
    {
        if (!pose1.header.stamp.isZero() && !pose2.header.stamp.isZero())
        {
            double distance =  sqrt((pose1.pose.position.x - pose2.pose.position.x) * (pose1.pose.position.x - pose2.pose.position.x)
                            +(pose1.pose.position.y - pose2.pose.position.y) * (pose1.pose.position.y - pose2.pose.position.y));
//            addGaussianNoise(distance, mean, stdDev);

            tf2::Quaternion quat1, quat2;
            tf2::fromMsg(pose1.pose.orientation, quat1);
            tf2::fromMsg(pose2.pose.orientation, quat2);
            double yaw1 = tf2::getYaw(quat1);
            if (yaw1 < 0)
                yaw1 += 2 * M_PI;
            double yaw2 = tf2::getYaw(quat2);
            if (yaw2 < 0)
                yaw2 += 2 * M_PI;

            double yaw_diff = yaw1 - yaw2;
            if (yaw_diff < 0)
                yaw_diff += 2 * M_PI;
            yaw_diff = 2*M_PI - yaw_diff;
//            addGaussianNoise(yaw_diff, mean, stdDev);
            yaw_diff *= 180 / M_PI;

            yaw1 *= 180 / M_PI;
            double x_diff = pose2.pose.position.x - pose1.pose.position.x;
            double y_diff = pose2.pose.position.y - pose1.pose.position.y;
            double bearing = atan2(y_diff , x_diff);
            if (bearing < 0)
                bearing += 2 * M_PI;
//            bearing = bearing * (180 / M_PI) - yaw1;
            bearing = bearing - yaw1 * (M_PI / 180);
            if (bearing < 0)
                bearing += 2 * M_PI;
//            std::cout << x_diff << y_diff << std::endl;
//            std::cout << bearing << std::endl;

//            bearing *= 180 / M_PI;
//            std::cout << bearing << std::endl;
//            addGaussianNoise(bearing, mean, stdDev);

            geometry_msgs::PointStamped measurements;
            measurements.header = pose1.header;
            measurements.point.x = distance * cos(bearing);
//            std::cout << measurements.point.x << std::endl;
            measurements.point.y = distance * sin(bearing);
            measurements.point.z = yaw_diff * M_PI / 180;

            pub2_.publish(measurements);
        }
    }

    void odometryPublish()
    {
        if (!twist_1.header.stamp.isZero() && !twist_2.header.stamp.isZero())
        {
            static ros::Time previousTime;
            static geometry_msgs::PointStamped previous_odom;

            ros::Time current_time = twist_2.header.stamp;

//            cout << previousTime << endl;
            if (previousTime.isZero())
            {
                previousTime = current_time;

                previous_odom.header.stamp = current_time;
                previous_odom.point.x = 0.0;
                previous_odom.point.y = 2.0;
                previous_odom.point.z = 0.0;
                pub3_.publish(previous_odom);

//                cout << previousTime << endl;
                return;
            }
            ros::Duration duration = current_time - previousTime;
            double delta_t = duration.toSec();
//            cout << "delta_t is : " << delta_t << endl;
//            cout << twist_2.twist.linear.x << ", " << twist_2.twist.angular.z << endl;
            if(delta_t != 0)
            {


                current_odom.header.stamp = current_time;
                current_odom.point.x = previous_odom.point.x + delta_t * ( twist_2.twist.linear.x * cos(previous_odom.point.z) + previous_odom.point.y * twist_1.twist.angular.z - twist_1.twist.linear.x);
                current_odom.point.y = previous_odom.point.y + delta_t * ( twist_2.twist.linear.x * sin(previous_odom.point.z) - previous_odom.point.x * twist_1.twist.angular.z);
                current_odom.point.z = previous_odom.point.z + delta_t * ( twist_2.twist.angular.z - twist_1.twist.angular.z);

//                cout << "previous_odom is " << previous_odom.point.x << ", " << previous_odom.point.y << ", " << previous_odom.point.z << endl;
//                cout << "current_odom is " << current_odom.point.x << ", " << current_odom.point.y << ", " << current_odom.point.z << endl;

                pub3_.publish(current_odom);

                previous_odom = current_odom;
                previousTime = current_time;

                // case
            }

        }
    }


    void addGaussianNoise(double& value, double mean, double stdDev)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> distribution(mean, stdDev);

        value += distribution(gen);
    }

    void run(){
        while (ros::ok()){
            // spin to process callbacks and naintain communication
            ros::spinOnce();

            // sleep to achieve the desired publishing rate (2.5Hz)
            rate_.sleep();
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "relative_robot_measurements");
    NoisyMeasurements my_node;
    ROS_INFO("Init!");
    my_node.run();
    return 0;
}
