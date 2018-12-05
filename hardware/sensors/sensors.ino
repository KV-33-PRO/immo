#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define RATE_MS 10

ros::NodeHandle nh;

sensor_msgs::Range ir_fl_msg;
sensor_msgs::Range ir_fr_msg;
sensor_msgs::Range ir_rl_msg;
sensor_msgs::Range ir_rr_msg;

ros::Publisher ir_fl_pub( "range/front/f_left_ir", &ir_fl_msg);
ros::Publisher ir_fr_pub( "range/front/f_right_ir", &ir_fr_msg);
ros::Publisher ir_rl_pub( "range/rear/r_left_ir", &ir_rl_msg);
ros::Publisher ir_rr_pub( "range/rear/r_right_ir", &ir_rr_msg);

unsigned long last_ms;

void setup()
{
  nh.getHardware()->setBaud(500000);
  nh.initNode();

  nh.advertise(ir_fl_pub);
  nh.advertise(ir_fr_pub);
  nh.advertise(ir_rl_pub);
  nh.advertise(ir_rr_pub);
  
  ir_fl_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_fl_msg.header.frame_id =  "/f_left_ir";
  ir_fl_msg.field_of_view = 0.01;
  ir_fl_msg.min_range = 0.03;
  ir_fl_msg.max_range = 0.8;

  ir_fr_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_fr_msg.header.frame_id =  "/f_right_ir";
  ir_fr_msg.field_of_view = 0.01;
  ir_fr_msg.min_range = 0.03;
  ir_fr_msg.max_range = 0.8;

  ir_rl_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_rl_msg.header.frame_id =  "/r_right_ir";
  ir_rl_msg.field_of_view = 0.01;
  ir_rl_msg.min_range = 0.03;
  ir_rl_msg.max_range = 0.8;

  ir_rr_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_rr_msg.header.frame_id =  "/r_right_ir";
  ir_rr_msg.field_of_view = 0.01;
  ir_rr_msg.min_range = 0.03;
  ir_rr_msg.max_range = 0.8;
}

void loop()
{
  if((millis() - last_ms) > RATE_MS){
    last_ms = millis();

    ir_fl_msg.range = getIRRange(0);
    ir_fl_msg.header.stamp = nh.now();
    ir_fl_pub.publish(&ir_fl_msg);

    ir_fr_msg.range = getIRRange(1);
    ir_fr_msg.header.stamp = nh.now();
    ir_fr_pub.publish(&ir_fr_msg);
    
    ir_rl_msg.range = getIRRange(2);
    ir_rl_msg.header.stamp = nh.now();
    ir_rl_pub.publish(&ir_fl_msg);

    ir_rr_msg.range = getIRRange(3);
    ir_rr_msg.header.stamp = nh.now();
    ir_rr_pub.publish(&ir_fr_msg);
  }

  nh.spinOnce();
}

float getIRRange(int analogPin) {
  int data;
  data = analogRead(analogPin)/4;
  if(data < 10)
  return 254;
  data = 1309/(data-3);
  return (data - 1)/100; //convert to meters
  
}
