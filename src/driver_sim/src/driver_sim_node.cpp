#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define NUM_JOINTS         3
#define WHEEL_DIAMETER     0.151

double cmd_angular;
double cmd_linear;



double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void cmdCallback(const geometry_msgs::Twist &msg)
{
    cmd_angular = msg.angular.z + fRand(-0.01, 0.01);
    cmd_linear = (msg.linear.x / (WHEEL_DIAMETER/2));
    if(cmd_linear != 0) {
        cmd_linear += fRand(-0.02, 0.02);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driver_sim_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, cmdCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate r(20);
    sensor_msgs::JointState msg;
    msg.header.frame_id =  "/driver_states";
    msg.name.resize(3);
    msg.position.resize(3);
    msg.velocity.resize(3);
    msg.effort.resize(3);
    msg.name[0] = "left_wheel";
    msg.name[1] = "right_wheel";
    msg.name[2] = "rudder";
    while(ros::ok()){
        ros::spinOnce();
        ros::Duration dt =  ros::Time::now() - msg.header.stamp;
        msg.header.stamp =  ros::Time::now();
        msg.position[0] = cmd_linear * dt.toSec();
        msg.position[1] = cmd_linear * dt.toSec();
        msg.position[2] = cmd_angular;
        msg.velocity[0] = cmd_linear;
        msg.velocity[1] = cmd_linear;
        msg.velocity[2] = 0;
        pub.publish(msg);
        r.sleep();
    }
    return 0;
}
