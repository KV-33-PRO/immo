#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define RATE_MS 10

ros::NodeHandle nh;

sensor_msgs::Range ir_l_msg;
sensor_msgs::Range ir_r_msg;
sensor_msgs::Range snr_l_msg;
sensor_msgs::Range snr_r_msg;


ros::Publisher ir_l_pub( "range/front/left_ir", &ir_l_msg);
ros::Publisher ir_r_pub( "range/front/right_ir", &ir_r_msg);
ros::Publisher snr_l_pub( "range/front/left_snr", &snr_l_msg);
ros::Publisher snr_r_pub( "range/front/right_snr", &snr_r_msg);

unsigned long last_ms;


void setup()
{ 
  nh.getHardware()->setBaud(500000);
  nh.initNode();

  nh.advertise(ir_l_pub);
  nh.advertise(ir_r_pub); 
  
  ir_l_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_l_msg.header.frame_id =  "/left_ir";
  ir_l_msg.field_of_view = 0.01;
  ir_l_msg.min_range = 0.03;
  ir_l_msg.max_range = 0.8;

  ir_r_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_r_msg.header.frame_id =  "/right_ir";
  ir_r_msg.field_of_view = 0.01;
  ir_r_msg.min_range = 0.03;
  ir_r_msg.max_range = 0.8;

  //ir_l_msg.radiation_type = sensor_msgs::Range::ULTRASONIC;
  ir_l_msg.header.frame_id =  "/left_snr";
  ir_l_msg.field_of_view = 0.10000000149;
  ir_l_msg.min_range = 0.3;
  ir_l_msg.max_range = 4.0;

  //ir_r_msg.radiation_type = sensor_msgs::Range::ULTRASONIC;
  ir_r_msg.header.frame_id =  "/right_snr";
  ir_r_msg.field_of_view = 0.10000000149;
  ir_r_msg.min_range = 0.3;
  ir_r_msg.max_range = 4.0;
}

void loop()
{
  if((millis() - last_ms) > RATE_MS){
    last_ms = millis();

    ir_l_msg.range = getIRRange(0);
    ir_l_msg.header.stamp = nh.now();
    ir_l_pub.publish(&ir_l_msg);

    ir_r_msg.range = getIRRange(1);
    ir_r_msg.header.stamp = nh.now();
    ir_r_pub.publish(&ir_r_msg); 

    snr_r_msg.range = getSNRRange(3,4);
    snr_r_msg.header.stamp = nh.now();
    snr_r_pub.publish(&snr_r_msg); 

    snr_l_msg.range = getSNRRange(5,6);
    snr_l_msg.header.stamp = nh.now();
    snr_l_pub.publish(&snr_l_msg); 
 
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

float getSNRRange(int echoPin, int trigPin) {
  float duration;
  float dist;
  digitalWrite(trigPin, LOW); // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);   // Reads the echoPin, returns the sound wave travel time in microseconds
  dist = duration*0.034/2; // Calculating the distance in cm
  return dist*100;
  }
