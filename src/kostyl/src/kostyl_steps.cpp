#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

int step0(nav_msgs::Odometry &odom, geometry_msgs::Twist::Ptr twist) {
    if(odom.pose.pose.position.x < 2.7) {
        twist->angular.z = 0;
        twist->linear.x = 0.5;
        return 0;
    } else {
        return 1;
    }
}

int step1(nav_msgs::Odometry &odom, geometry_msgs::Twist::Ptr twist) {
    if(odom.pose.pose.orientation.w > 0.75) {
        twist->angular.z = 0.3;
        twist->linear.x = 0.5;
        return 1;
    } else {
        return 2;
    }
}

int step2(nav_msgs::Odometry &odom, geometry_msgs::Twist::Ptr twist) {
    if(odom.pose.pose.position.y < 4) {
        twist->angular.z = 0;
        twist->linear.x = 0.5;
        return 2;
    } else {
        return 3;
    }
}

/*
int step3(nav_msgs::Odometry &odom, geometry_msgs::Twist::Ptr twist) {
    if(odom.pose.pose.position.y < 4) {
        twist->angular.z = 0;
        twist->linear.x = 0.5;
        return 2;
    } else {
        return 3;
    }
}
*/
