#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
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
        kp = 3.6;
        ka = 0.8;
        kb = 1;
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

    virtual void SendMessage() = 0;

};

class TurtleListener: public Listener
{

private:

    ros::Publisher pub;

public:

    TurtleListener(float goalx, float goaly, float goalTheta)
    : Listener(goalx, goaly, goalTheta)
    {
        ros::NodeHandle nh;
        pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    }

    void UpdatePose(const turtlesim::Pose &msg)
    {
        this->x = msg.x;
        this->y = msg.y;
        this->theta = msg.theta;
        toPolar();
        setNextPolarParameters();
        setActualParameters();
    }

    void SendMessage()
    {
        geometry_msgs::Twist msg;
        msg.linear.x =  v;
        msg.angular.z = v * tan(gamma);
        pub.publish(msg);
    }
};

class CarListener: public Listener
{

private:

    ros::Publisher pubVel;
    ros::Publisher pubSteer;

public:

    CarListener(float goalx, float goaly, float goalTheta)
    : Listener(goalx, goaly, goalTheta)
    {
        ros::NodeHandle nh;
        pubVel = nh.advertise<std_msgs::Float32>("AutoNOMOS_mini/manual_control/velocity", 1000);
        pubSteer = nh.advertise<std_msgs::Float32>("AutoNOMOS_mini/manual_control/steering", 1000);
    }

    void UpdatePose(const tf2_msgs::TFMessage &msg)
    {
        this->x = msg.transforms[0].transform.translation.x;
        this->y = msg.transforms[0].transform.translation.y;
        this->theta = msg.transforms[0].transform.rotation.z;
        toPolar();
        setNextPolarParameters();
        setActualParameters();
    }

    void SendMessage()
    {
        std_msgs::Float32 msgv;
        msgv.data = v;
        std_msgs::Float32 msgs;
        msgs.data = gamma;
        pubVel.publish(msgv);
        pubSteer.publish(msgs);
    }
};

void sendMessage(Listener &l)
{
    ros::Rate rate(1); 
    while(ros::ok())
    {
        ros::spinOnce();

        ROS_INFO_STREAM("Position: x = " << l.x << " y = " << l.y << " theta = " << l.theta);
        ROS_INFO_STREAM("Polar: p = " << l.ro << " alpha = " << l.alpha << " beta = " << l.beta);

        l.SendMessage();

        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");

    ros::NodeHandle nh;

    //TurtleListener l(9,9,0);
    //ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &TurtleListener::UpdatePose, &l);

    CarListener l(9,0,0);
    ros::Subscriber sub = nh.subscribe("tf", 1000, &CarListener::UpdatePose, &l);

    ros::spinOnce();

    sendMessage(l);

}