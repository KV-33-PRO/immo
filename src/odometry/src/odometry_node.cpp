#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include "odometry/SetPose.h"

double aw  = 0.333;   // межосевое расстояние (в метрах)
double d = 0.151;     // диаметр колеса (в метрах)

double x  = 0.0;
double y  = 0.0;
double th = 0.0;

double vx  = 0.0;   // m/s
double vy  = 0.0;   // m/s
double vth = 0.0;   // radian/s

ros::Time prev_time;
bool prev_time_init = false;

double linear_speed = 0.0;
double linear_offset = 0.0;
double rudder_angle  = 0.0;

bool setPoseHandler(odometry::SetPose::Request  &req,
         odometry::SetPose::Response &res)
{
    x = req.pose.x;
    y = req.pose.y;
    th = req.pose.theta;
    return true;
}

void jointStatesCallback(const sensor_msgs::JointState& msg)
{
    if(msg.name.size() == 3 && msg.name[0].compare("left_wheel") == 0 && msg.name[1].compare("right_wheel") == 0 && msg.name[2].compare("rudder") == 0) {
        if(prev_time_init) {
            linear_speed = ((msg.velocity[0] + msg.velocity[1]) / 2) * (d / 2);
            linear_offset = ((msg.position[0] + msg.position[1]) / 2) * (d / 2);
            rudder_angle = msg.position[2];
            vth = linear_speed * tan(rudder_angle) / aw;
            ros::Duration dt = msg.header.stamp - prev_time;
            double x_dot = linear_speed * cos(th);
            double y_dot = linear_speed * sin(th);
            x += x_dot * dt.toSec();
            y += y_dot * dt.toSec();
            th += vth * dt.toSec();
        }
        prev_time = msg.header.stamp;
        prev_time_init = true;
        ROS_INFO("Rudder: %0.2f; V: %0.2f; S: %0.2f; X: %0.2f; Y: %0.2f; Th: %0.2f;", rudder_angle, linear_speed, linear_offset, x, y, th);
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 10, jointStatesCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::ServiceServer service = n.advertiseService("odometry_set_pose", setPoseHandler);

  boost::shared_ptr<tf::TransformBroadcaster> tf_pub;
  tf_pub.reset(new tf::TransformBroadcaster);

  ROS_INFO("Odometry initialized");

  ros::Rate r(20);

  while(ros::ok()){
    ros::spinOnce();

    // publish odometry message
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.frame_id = "odom";
    odom->header.stamp = ros::Time::now();
    odom->child_frame_id = "base_link";

    // Position
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.orientation.x = 0.0;
    odom->pose.pose.orientation.y = 0.0;
    odom->pose.pose.orientation.z = sin(th/2.0);
    odom->pose.pose.orientation.w = cos(th/2.0);

    // Position uncertainty
    odom->pose.covariance[0]  = 0.01; ///< x
    odom->pose.covariance[7]  = 0.01; ///< y
    odom->pose.covariance[35] = 0.01; ///< yaw

    // Velocity ("in the coordinate frame given by the child_frame_id")
    odom->twist.twist.linear.x = linear_speed;
    odom->twist.twist.linear.y = 0.0;
    odom->twist.twist.angular.z = vth;

    // Velocity uncertainty
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = odom->header.frame_id;
    tf.child_frame_id = odom->child_frame_id;
    tf.header.stamp = ros::Time::now();
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom->pose.pose.orientation;
    if (ros::ok()) {
        tf_pub->sendTransform(tf);
    }
    if (ros::ok()) {
        odom_pub.publish(odom);
    }

    r.sleep();
  }
  return 0;
}
