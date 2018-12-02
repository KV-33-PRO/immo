#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

double aw  = 0.7;   // межосевое расстояние (в метрах)
double d = 0.2;     // диаметр колеса (в метрах)
double g  = 90.0;   // угол поворота колес между положениями "вправо" - "влево" (в градусах)
int n = 10;         // число импульсов на один оборот колеса

double x  = 0.0;
double y  = 0.0;
double th = 0.0;

double vx  = 0.0;   // m/s
double vy  = 0.0;   // m/s
double vth = 0.0;   // radian/s

ros::Time prev_time;

double s  = 0.0;
double angular  = 0.0;

void jointStatesCallback(const sensor_msgs::JointState& msg)
{
  double dt = (msg.header.stamp - prev_time).toSec();
  if(dt < 10 && dt > 0) {
    angular = 0;
    s = 0;
    for(int i = 0; i < msg.name.size(); i++){
      if(msg.name[i].compare("left_wheel") == 0 & msg.name[i+1].compare("right_wheel") == 0) {
          s = (((msg.velocity[i]+msg.velocity[i+1])/2)/n)*M_PI*d;   //пройденный путь в метрах
          i++;
      } else if(msg.name[i].compare("rudder") == 0) {
          angular = ((msg.position[i]-1000)*(g/1000))*M_PI/180;   //преобразование значения с сервы(1000-2000) > градусы > радианы
      }
    }

    double delta_th;
    if(angular != 0) {
      delta_th = s/(aw/tan(angular));
    } else {
      delta_th = 0;
    }
    double delta_x = s * cos(th + delta_th/2);
    double delta_y = s * sin(th + delta_th/2);

    x += delta_x;
    y += delta_y;
    th += delta_th;

    vx = delta_x/dt;
    vy = delta_y/dt;
    vth = delta_th/dt;
  }
  prev_time = msg.header.stamp;
  ROS_INFO("Angular: %0.2f; Velocity: %0.2f; Time (dt): %0.4f; X: %0.2f; Y: %0.2f; Th: %0.2f", angular, s, dt, x, y, th);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 10, jointStatesCallback);
  ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50);

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
