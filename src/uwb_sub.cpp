#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/LinktrackAoaNode0.h>

class MyPubSub{
int cnt = 0;
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber uwbSub;
    geometry_msgs::PointStamped measurements;
    

public:
    MyPubSub()
    {
        uwbSub = nh.subscribe("nlink_linktrack_aoa_nodeframe0", 100, &MyPubSub::uwbCb, this);
        // ultraSub = nh.subscribe("ultra", 10, &MyPubSub::ultraCb, this);
        pub = nh.advertise<geometry_msgs::PointStamped>("aoa_measurements", 1);
    }


    void uwbCb(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr &msg)
    {   cnt +=1;
        if(cnt ==4)
        {
            if(!msg->nodes.empty())
            {
                ROS_INFO("msg LinktrackAoaNodeframe0 received, distance = %.4f", msg->nodes[0].dis);
                ROS_INFO("msg LinktrackAoaNodeframe0 received, angle = %.4f", msg->nodes[0].angle);

                const float aoa_range = msg->nodes[0].dis;
                const float aoa_angle = msg->nodes[0].angle;

                geometry_msgs::PointStamped measurements;
                measurements.header.stamp = ros::Time::now();
                measurements.point.x = aoa_range;
                measurements.point.y = aoa_angle;

                pub.publish(measurements);
            
            }
            cnt =0;
        }
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_sub_pub");
  MyPubSub my_pub;
  ros::spin();

  return 0;
}