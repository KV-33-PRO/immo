#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "kostyl_steps.cpp"

nav_msgs::Odometry odom;
sensor_msgs::LaserScan scan;
double d_front = 10;
double d_back = 10;
double d_left = 10;
double d_right = 10;

void odometryCallback(const nav_msgs::Odometry &msg)
{
    odom = msg;
}

void getDist(double &d, int f, int t) {
    for(int i = f; i < t; i++)
        if(scan.ranges[i] > 0 && d > scan.ranges[i])
            d = scan.ranges[i];
}

void laserCallback(const sensor_msgs::LaserScan &msg)
{
    scan = msg;
    d_front = 10;
    d_back = 10;
    d_left = 10;
    d_right = 10;
    getDist(d_front, 0, 10);
    getDist(d_front, 350, 359);
    getDist(d_left, 80, 100);
    getDist(d_back, 170, 190);
    getDist(d_right, 260, 280);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kostyl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("odom", 10, odometryCallback);
    ros::Subscriber sub2 = nh.subscribe("scan", 10, laserCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate r(50);
    geometry_msgs::Twist::Ptr twist(new geometry_msgs::Twist);
    int step = 0;
    while(ros::ok()){
        ros::spinOnce();
        /*
        switch(step) {
            case 0:
                step = step0(odom, twist);
                break;
            case 1:
                step = step1(odom, twist);
                break;
            case 2:
                step = step2(odom, twist);
                break;
            default:
                twist->angular.z = 0;
                twist->linear.x = 0;
                break;

        }

        pub.publish(twist);
        */
        double rr = 0;
        if(scan.range_min > 0)
            rr = scan.ranges[350];
        ROS_INFO("ODOM X: %0.2f; Y: %0.2f; Z: %0.2f; W: %0.2f; F: %0.2f; L: %0.2f; B: %0.2f; R: %0.2f;",
                 odom.pose.pose.position.x,
                 odom.pose.pose.position.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w,
                 d_front,
                 d_left,
                 d_back,
                 d_right);
        r.sleep();
    }
    return 0;
}
