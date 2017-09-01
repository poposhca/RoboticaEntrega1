#include <ros/ros.h>
#include <turtlesim/Pose.h>

void poseMessage(const turtlesim::Pose &msg)
{
    ROS_INFO_STREAM("Position: x = " << msg.x << ", y = " << msg.y << ", dir = " << msg.theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessage);
    ros::spin();
}