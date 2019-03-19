#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
geometry_msgs::PoseStamped target;

#define D_MAX 10

double d_front = D_MAX;
double d_back = D_MAX;
double d_left = D_MAX;
double d_right = D_MAX;

void odometryCallback(const nav_msgs::Odometry &msg)
{
    odom = msg;
}

void targetCallback(const geometry_msgs::PoseStamped &msg)
{
    target = msg;
}

void getDist(double &d, int f, int t) {
    for(int i = f; i < t; i++)
        if(scan.ranges[i] > 0 && d > scan.ranges[i])
            d = scan.ranges[i];
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    scan = msg;
    d_front = D_MAX;
    d_back = D_MAX;
    d_left = D_MAX;
    d_right = D_MAX;
    getDist(d_front, 0, 10);
    getDist(d_front, 350, 359);
    getDist(d_left, 80, 100);
    getDist(d_back, 170, 190);
    getDist(d_right, 260, 280);
}

void process(geometry_msgs::Twist::Ptr cmd);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "immo_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Subscriber sub1 = nh.subscribe("odom", 10, odometryCallback);
    ros::Subscriber sub2 = nh.subscribe("scan", 10, laserCallback);
    ros::Subscriber sub3 = nh.subscribe("target_pose", 10, targetCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate r(50);

    geometry_msgs::Twist::Ptr twist(new geometry_msgs::Twist);
    while(ros::ok()){
        ros::spinOnce();
        twist->angular.z = 0;
        twist->linear.x = 0;
        process(twist);
        cmd_pub.publish(twist);
        /*
        ROS_INFO("ODOM X: %0.2f; Y: %0.2f; Z: %0.2f; W: %0.2f; F: %0.2f; L: %0.2f; B: %0.2f; R: %0.2f;",
                 odom.pose.pose.position.x,
                 odom.pose.pose.position.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w,
                 d_front,
                 d_left,
                 d_back,
                 d_right);
        */
        r.sleep();
    }
    return 0;
}

/*
void drive(uint8_t from, uint8_t to, geometry_msgs::Twist::Ptr cmd) {

    GraphNode from_node = graph.nodes[from];
    GraphNode to_node = graph.nodes[to];
    tfScalar ang = tf::tfAngle(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0), tf::Vector3(to_node.x, to_node.y, 0));
    ROS_INFO("Target angle: %0.2f", ang);
    /*
    double lr = d_left - d_right;
    if(lr > 0.3) lr = 0.1;
    if(lr < -0.3) lr = -0.1;
    cmd->angular.z = lr;
    cmd->linear.x = 0.3;
}
    */


void process(geometry_msgs::Twist::Ptr cmd) {
    ros::Duration target_dt = ros::Time::now() - target.header.stamp;
    ros::Duration odom_dt = ros::Time::now() - odom.header.stamp;
    //ROS_INFO("Target time: %0.2f", dt.toSec());
    if(target_dt.toSec() < 0.1 &&  odom_dt.toSec() < 0.1) {
        double target_yaw = atan2(target.pose.position.y - odom.pose.pose.position.y, target.pose.position.x - odom.pose.pose.position.x);
        double robot_yaw = tf::getYaw(odom.pose.pose.orientation);
        double yaw = target_yaw - robot_yaw;
        if(abs(yaw) < 1.7) {
            cmd->angular.z = yaw;
            cmd->linear.x = 0.3;
        }
    }
}
