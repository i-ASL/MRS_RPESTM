#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "multiple_turtlebots_sim/SyncMsg.h"

using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher sync_data_pub_;

void callback(const PointStampedConstPtr& odom, const PointStampedConstPtr& meas,
              const PointStampedConstPtr& gt, const PointStampedConstPtr& case0,
              const PointStampedConstPtr& case1, const PointStampedConstPtr& case2,
              const PointStampedConstPtr& case3, const TwistStampedConstPtr& case4)
{
    static ros::Time previousTime;

    multiple_turtlebots_sim::SyncMsg sync_msg;
    sync_msg.header.stamp = odom->header.stamp;
    sync_msg.header.frame_id = "0";
    sync_msg.odom_x = odom->point.x;
    sync_msg.odom_y = odom->point.y;
    sync_msg.odom_theta = odom->point.z;
    sync_msg.meas_rho = meas->point.x;
    sync_msg.meas_beta = meas->point.y;
    sync_msg.meas_theta = meas->point.z;
    sync_msg.gt_rho = gt->point.x;
    sync_msg.gt_beta = gt->point.y;
    sync_msg.gt_theta = gt->point.z;
    sync_msg.case0_x = case0->point.x;
    sync_msg.case0_y = case0->point.y;
    sync_msg.case0_t = case0->point.z;
    sync_msg.case1_x = case1->point.x;
    sync_msg.case1_y = case1->point.y;
    sync_msg.case1_t = case1->point.z;
    sync_msg.case2_x = case2->point.x;
    sync_msg.case2_y = case2->point.y;
    sync_msg.case2_t = case2->point.z;
    sync_msg.case3_x = case3->point.x;
    sync_msg.case3_y = case3->point.y;
    sync_msg.case3_t = case3->point.z;
    sync_msg.case4_x = case4->twist.linear.x;
    sync_msg.case4_y = case4->twist.linear.y;
    sync_msg.case4_t = case4->twist.linear.z;
    sync_msg.case4_vj = case4->twist.angular.x;
    sync_msg.case4_wj = case4->twist.angular.y;

    ros::Time current_time = sync_msg.header.stamp;

    if (previousTime.isZero())
    {
        previousTime = current_time;
        sync_data_pub_.publish(sync_msg);
        return;
    }
    ros::Duration duration = current_time - previousTime;
    double delta_t = duration.toSec();
    sync_msg.delta_t = delta_t;
    std::cout << delta_t << std::endl;
    previousTime = current_time;

    sync_data_pub_.publish(sync_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;
    ROS_INFO("Init!");

    message_filters::Subscriber<PointStamped> odom_sub(nh, "/odometry", 1);
    message_filters::Subscriber<PointStamped> meas_sub(nh, "/noisy_measurements", 1);
    message_filters::Subscriber<PointStamped> gt_sub(nh, "/ground_truth", 1);
    message_filters::Subscriber<PointStamped> case0_sub(nh, "/case_zero", 1);
    message_filters::Subscriber<PointStamped> case1_sub(nh, "/case_one", 1);
    message_filters::Subscriber<PointStamped> case2_sub(nh, "/case_two", 1);
    message_filters::Subscriber<PointStamped> case3_sub(nh, "/case_three", 1);
    message_filters::Subscriber<TwistStamped> case4_sub(nh, "/case_four", 1);


    typedef sync_policies::ApproximateTime<PointStamped, PointStamped, PointStamped, PointStamped,
            PointStamped, PointStamped, PointStamped, TwistStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, meas_sub, gt_sub, case0_sub,
                                    case1_sub, case2_sub, case3_sub, case4_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8));

    sync_data_pub_ = nh.advertise<multiple_turtlebots_sim::SyncMsg>("sync_data", 1);

    ros::spin();

    return 0;
}

