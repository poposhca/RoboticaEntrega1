#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <string.h>

//INPUT:
//argv[1] distancia d a mover de la tortuga
//argv[2] Velocidad maxima de la tortuga
int main(int argc, char **argv)
{
    //Read user input
    if(argc != 3)
    {
        ROS_INFO_STREAM("USAGE: PACK distance max_velocity");
        exit(9);
    }
    float d = atof(argv[1]);
    float vmax = atof(argv[2]);

    //Calcula parametros para el loop
    float deltaTime;
    float t = d / vmax;

    //Suscribir
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    srand(time(0));
    
    ros::Rate rate(10);

    //Primer segundo, aceleracion
    deltaTime = 0;
    float acc = vmax*0.1;
    float vacc = 0;
    while(ros::ok() && deltaTime < 1)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = vacc;
        vacc += acc;
        ROS_INFO_STREAM("linear: " << msg.linear.x << "  Angular: " << msg.linear.z);
        pub.publish(msg);
        deltaTime += 0.1; 
        rate.sleep();
    }

    if(t-2 > 0)
    {
        deltaTime = 0;
        while(ros::ok() && deltaTime <= t-2)
        {
            geometry_msgs::Twist msg;
            msg.linear.x = vmax;
            ROS_INFO_STREAM("linear: " << msg.linear.x << "  Angular: " << msg.linear.z);
            pub.publish(msg);
            deltaTime += 0.1; 
            rate.sleep();
        }
    }

    //Ultimo segundo de aceleracion
    deltaTime = 0;
    acc = acc * -1;
    while(ros::ok() && deltaTime < 1)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = vacc;
        vacc += acc;
        ROS_INFO_STREAM("linear: " << msg.linear.x << "  Angular: " << msg.linear.z);
        pub.publish(msg);
        deltaTime += 0.1; 
        rate.sleep();
    }
}