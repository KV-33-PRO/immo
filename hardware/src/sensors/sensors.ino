#include <ros.h>
#include <sensor_msgs/Range.h>
#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

#define RATE_MS       20

ros::NodeHandle nh;

sensor_msgs::Range snr_l_msg;
sensor_msgs::Range snr_r_msg;

ros::Publisher snr_l_pub("range/front/left_snr", &snr_l_msg);
ros::Publisher snr_r_pub("range/front/right_snr", &snr_r_msg);

unsigned long last_ms;
Ultrasonic sonar_l(PA15, PB3);
Ultrasonic sonar_r(PB4, PB5);

SimpleKalmanFilter kf_l(2, 2, 0.01);
SimpleKalmanFilter kf_r(2, 2, 0.01);

void setup()
{
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(snr_l_pub);
  nh.advertise(snr_r_pub);

  snr_l_msg.header.frame_id =  "/left_snr";
  snr_l_msg.field_of_view = 0.10000000149;
  snr_l_msg.min_range = 0.3;
  snr_l_msg.max_range = 3.5;

  snr_r_msg.header.frame_id =  "/right_snr";
  snr_r_msg.field_of_view = 0.10000000149;
  snr_r_msg.min_range = 0.3;
  snr_r_msg.max_range = 3.5;
}

void loop()
{
  if ((millis() - last_ms) > RATE_MS) {
    last_ms = millis();

    snr_l_msg.range = getSNRRange(0);
    snr_l_msg.header.stamp = nh.now();
    snr_l_pub.publish(&snr_l_msg);

    snr_r_msg.range = getSNRRange(1);
    snr_r_msg.header.stamp = nh.now();
    snr_r_pub.publish(&snr_r_msg);

  }
  nh.spinOnce();
}

float getSNRRange(int side) {
  if (side == 0) {
    return kf_l.updateEstimate(sonar_l.read()/100.0);
  }
  else
  {
    return kf_r.updateEstimate(sonar_r.read()/100.0);
  }
}
