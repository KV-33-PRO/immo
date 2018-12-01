#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

double x  = 0.0;
double y  = 0.0;
double th = 0.0;

double vx  = 0.0;
double vy  = 0.0;
double vth = 0.0;

ros::Time prev_time;

double v  = 0.0;
double a  = 0.0;
// межосевое расстояние
double L  = 0.7;

void jointStatesCallback(const sensor_msgs::JointState& msg)
{
  double dt = (msg.header.stamp - prev_time).toSec();
  if(dt < 10 && dt > 0) {
    a = 0;
    v = 0;
    for(int i = 0; i < msg.name.size(); i++){
      if(msg.name[i].compare("steering_joint") == 0) {
        a = (msg.position[i]/10)*M_PI/180;
      } else if(msg.name[i].compare("velocity_joint") == 0) {
        v = msg.velocity[i]/33000.0;
      }
    }
    double dist = v;
    double delta_th;
    if(a != 0) {
      double R = L/tan(a);
      delta_th = dist/R;
    } else {
      delta_th = 0;
    }

    double delta_x = dist * cos(th + delta_th/2);
    double delta_y = dist * sin(th + delta_th/2);

    x += delta_x;
    y += delta_y;
    th += delta_th;

    vx = delta_x/dt;
    vy = delta_y/dt;
    vth = delta_th/dt;
  }
  prev_time = msg.header.stamp;
  ROS_INFO("Steering: %0.2f; Velocity: %0.2f; dt: %0.4f; X: %0.2f; Y: %0.2f; Th: %0.2f", a, v, dt, x, y, th);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "futs_qt_odometry_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/joint_states", 10, jointStatesCallback);
  ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/odometry", 10);

  ros::Rate r(100);
  while(ros::ok()){
    ros::spinOnce();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    pub.publish(odom);
    r.sleep();
  }
  return 0;
}
