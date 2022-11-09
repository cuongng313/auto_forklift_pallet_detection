#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::cout << " x " << msg->linear.x << std::endl;
    std::cout << " yaw " << msg->angular.z << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, chatterCallback);
    // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}