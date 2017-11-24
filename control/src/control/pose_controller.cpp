#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <string.h>

class Listener
{

public:
    
    int x;

    void UpdatePose(const turtlesim::Pose &msg)
    {
        x = msg.x;
    }
};

void sendMessage(Listener &l)
{
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    ros::Rate rate(1); 
    while(ros::ok())
    {
        ros::spinOnce();
        if(l.x < 9)
        {
            ROS_INFO_STREAM("Position: x = " << l.x);
            geometry_msgs::Twist msg;
            msg.linear.x =  1;
            pub.publish(msg);
        }
        else
        {
            ROS_INFO_STREAM("Tortuga Detenida");
        }

        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");

    ros::NodeHandle nh;

    Listener l;
    l.x = 0;
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &Listener::UpdatePose, &l);
    ros::spinOnce();

    sendMessage(l);

}