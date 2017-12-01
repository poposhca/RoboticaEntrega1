#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <angles/angles.h>

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
        kp = 0.6;
        //ka > kp
        ka = 1.6;
        //Kb < 0
        kb = -0.55;
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
        beta =  - theta - alpha + angles::from_degrees(goalTheta);
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

    void UpdateNextPose(const geometry_msgs::Pose2D &msg)
    {
        this->goalx = msg.x;
        this->goaly = msg.y;
        this->goalTheta = msg.theta;
    }

    void UpdatePose(const tf2_msgs::TFMessage &msg)
    {
        this->x = msg.transforms[0].transform.translation.x;
        this->y = msg.transforms[0].transform.translation.y;
        this->theta = msg.transforms[0].transform.rotation.z;
    }

    void SendMessage()
    {
        std_msgs::Float32 msgv;
        msgv.data = v;
        std_msgs::Float32 msgs;
        msgs.data = angles::to_degrees(gamma);
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

        l.toPolar();
        l.setActualParameters();

        ROS_INFO_STREAM("Position: x = " << l.x << " y = " << l.y << " theta = " << l.theta);
        ROS_INFO_STREAM("Polar: p = " << l.ro << " alpha = " << l.alpha << " beta = " << l.beta);
        ROS_INFO_STREAM("Params: V = " << l.v << " gamma = " << l.gamma);

        l.SendMessage();

        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");

    ros::NodeHandle nh;

    //TurtleListener l(2,2,0);
    //ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &TurtleListener::UpdatePose, &l);

    CarListener l(0,0,0);
    ros::Subscriber sub1 = nh.subscribe("tf", 1000, &CarListener::UpdatePose, &l);
    ros::Subscriber sub2 = nh.subscribe("robot/next_pose", 1000, &CarListener::UpdateNextPose, &l);

    ros::spinOnce();

    sendMessage(l);

}