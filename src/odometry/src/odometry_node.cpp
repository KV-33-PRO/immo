#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

double aw  = 0.333;   // межосевое расстояние (в метрах)
double d = 0.151;     // диаметр колеса (в метрах)

double x  = 0.0;
double y  = 0.0;
double th = 0.0;

double vx  = 0.0;   // m/s
double vy  = 0.0;   // m/s
double vth = 0.0;   // radian/s

ros::Time prev_time;

double dist  = 0.0;
double angular  = 0.0;


void jointStatesCallback(const sensor_msgs::JointState& msg)
{
  double dt = (msg.header.stamp - prev_time).toSec();
  if(dt < 10 && dt > 0) {
    angular = 0;
    dist = 0;
    for(int i = 0; i < msg.name.size(); i++){
      if(msg.name[i].compare("left_wheel") == 0 & msg.name[i+1].compare("right_wheel") == 0) {
          dist = ((msg.velocity[i]+msg.velocity[i+1])/2)*dt*(d/2);   // Перевод рад/с > рад > метры   (пройденный путь)
          i++;
      } else if(msg.name[i].compare("rudder") == 0) {
          angular = msg.position[i];  //радиан
      }
    }

    double delta_th;
    if(angular != 0) {
      delta_th = dist/(aw/tan(angular));
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
  ROS_INFO("Angular: %0.2f; Velocity: %0.2f; Time (dt): %0.4f; X: %0.2f; Y: %0.2f; Th: %0.2f", angular, dist, dt, x, y, th);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 10, jointStatesCallback);
  ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate r(50);
  while(ros::ok()){
    ros::spinOnce();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);


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
