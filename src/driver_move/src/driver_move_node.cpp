#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

class DriverMove{
private:
    double start_pose;
    double speed_cmd_vel_pub;
    double speed;
    double distantion;
    ros::Time start_time;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber odom_sub;
    ros::NodeHandle nh;
public:
    DriverMove(double speed_move, double dist, ros::NodeHandle &n){
        this->speed_cmd_vel_pub = speed_move;
        this->distantion = dist;
        this->start_pose = 0.0;
        speed = 0.0;
        nh = n;
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub = nh.subscribe("odom", 10, &DriverMove::odomCallBack, this);
    }

    void moveGoal(){
        ros::spinOnce();
        geometry_msgs::Twist vel_msg;
        ros::Rate r(10);
        vel_msg.linear.x = speed;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;

        cmd_vel_pub.publish(vel_msg);
        r.sleep();
    }

    void odomCallBack(const nav_msgs::Odometry &o_msg){
        bool info = true;
        if(start_pose==0.0){
            start_pose = o_msg.pose.pose.position.x;
            start_time = ros::Time::now();
        }
        if(o_msg.pose.pose.position.x <= distantion+start_pose){
            speed = speed_cmd_vel_pub;
        } else {
            speed = 0.0;
            info=false;
        }
        if(info){
        double t = ros::Time::now().toSec()-start_time.toSec();
        double dist = o_msg.pose.pose.position.x-start_pose;
        double sp = dist/t;
        ROS_INFO("dist: %.2f m, time: %.2f sec, speed: %.2f m/s", dist, t, sp);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "driver_move_node");
    ros::NodeHandle n;
    DriverMove move(0.5, 1.0, n);
    while(ros::ok()){
    move.moveGoal();
    }
    return(0);
}
