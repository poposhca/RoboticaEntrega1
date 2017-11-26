#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

class Listener
{

public:

    //Actual pose
    float x;
    float y;
    float theta;

    //Goal pose
    float goalx;
    float goaly;
    float goalTheta;

    //Polar coordenates
    float ro;
    float alpha;
    float beta;

    //Actual parameters
    float v;
    float gamma;

    //Control parameters
    float kp;
    float ka;
    float kb;
    
    Listener(float goalx, float goaly, float goalTheta)
    {
        this->goalx = goalx;
        this->goaly = goaly;
        this->goalTheta = goalTheta;
        kp = 1.5;
        ka = 0.8;
        kb = 0.8;
        v = 0;
        gamma = 0;
    }

    void toPolar()
    {
        //Calcular ro
        float dx = pow(x - goalx, 2);
        float dy = pow(y - goaly, 2);
        ro = sqrt(dx + dy);

        //Calcular alpha
        alpha = atan(dy / dx) - theta;

        //Calcular beta
        beta = -theta - alpha;
    }

    void setNextPolarParameters()
    {
        ro = kp * cos(alpha);
        alpha = kp * sin(alpha) - ka * alpha - kb * beta;
        beta = -kp * sin(ro);
    }

    void setActualParameters()
    {
        v = kp * ro;
        gamma = ka * alpha + kb * beta;
    }

};

class TurtleListener: public Listener
{

public:

    TurtleListener(float goalx, float goaly, float goalTheta)
    : Listener(goalx, goaly, goalTheta)
    {}

    void UpdatePose(const turtlesim::Pose &msg)
    {
        this->x = msg.x;
        this->y = msg.y;
        this->theta = msg.theta;
        toPolar();
        setNextPolarParameters();
        setActualParameters();
    }
};

void sendMessage(Listener &l)
{
    
    ros::NodeHandle nh;
    //Prueba tortuga
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    //Modelo Gazebo
    //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/AutoNOMOS_mini/manual_control/velocity", 1000);

    ros::Rate rate(1); 
    while(ros::ok())
    {
        ros::spinOnce();
        //TODO quitar esta validacion
        if(l.x < 10)
        {
            ROS_INFO_STREAM("Position: x = " << l.x << " y = " << l.y << " theta = " << l.theta);
            ROS_INFO_STREAM("Polar: p = " << l.ro << " alpha = " << l.alpha << " beta = " << l.beta);
            geometry_msgs::Twist msg;
            msg.linear.x =  l.v;
            msg.angular.z = l.gamma;
            pub.publish(msg);

            /*std_msgs::Float32 msg;
            msg.data = 5.0f;
            pub.publish(msg);*/
        }
        else
        {
            ROS_INFO_STREAM("FINAL POSITION");
            ROS_INFO_STREAM("Position: x = " << l.x << " y = " << l.y << " theta = " << l.theta);
            ROS_INFO_STREAM("Polar: p = " << l.ro << " alpha = " << l.alpha << " beta = " << l.beta);
        }

        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");

    ros::NodeHandle nh;

    TurtleListener l(9,9,0);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &TurtleListener::UpdatePose, &l);
    ros::spinOnce();

    sendMessage(l);

}