#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>  

#define RATE_MS 10

ros::NodeHandle nh;

sensor_msgs::Range snr_l_msg;
sensor_msgs::Range snr_r_msg;

ros::Publisher snr_l_pub( "range/front/left_snr", &snr_l_msg);
ros::Publisher snr_r_pub( "range/front/right_snr", &snr_r_msg);

unsigned long last_ms;
NewPing sonar_l(4, 3); 
NewPing sonar_r(6, 5); 

void setup()
{ 

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(snr_l_pub);
  nh.advertise(snr_r_pub);  

  snr_l_msg.header.frame_id =  "/left_snr";
  snr_l_msg.field_of_view = 0.10000000149;
  snr_l_msg.min_range = 0.3;
  snr_l_msg.max_range = 4.0;

  snr_r_msg.header.frame_id =  "/right_snr";
  snr_r_msg.field_of_view = 0.10000000149;
  snr_r_msg.min_range = 0.3;
  snr_r_msg.max_range = 4.0;
}

void loop()
{
  if((millis() - last_ms) > RATE_MS){
    last_ms = millis();

    snr_r_msg.range = getSNRRange(1);
    snr_r_msg.header.stamp = nh.now();
    snr_r_pub.publish(&snr_r_msg); 

    snr_l_msg.range = getSNRRange(0);
    snr_l_msg.header.stamp = nh.now();
    snr_l_pub.publish(&snr_l_msg); 
 
  }

  nh.spinOnce();
}

float getSNRRange(int number) {
float duration;
float dist;
  if (number == 0)
    duration = sonar_l.ping();
  else duration = sonar_r.ping();
  dist = duration / US_ROUNDTRIP_CM;
  return dist/100;
  }
