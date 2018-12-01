#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS          20
#define MOTOR_PIN_1      9
#define MOTOR_PIN_2      10
#define RUDDER_PIN       8
#define ENCODER_LEFT     2
#define ENCODER_RIGHT    3
#define NUM_JOINTS       3

bool encodersFlag[2] = {LOW, LOW}; //0-LEFT, 1-RIGHT
char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel", "rudder"};
float state_pos[NUM_JOINTS] = {0, 0, 0};
float state_vel[NUM_JOINTS] = {0, 0, 0};
float state_eff[NUM_JOINTS] = {0, 0, 0};
unsigned long last_ms;

ros::NodeHandle nh;

Servo rudder;
float linear;
float angular;

void drive_cb(const geometry_msgs::Twist& cmd_vel){
  linear = map(cmd_vel.linear.x * 1000, -1000, 1000, -255, 255);
  angular = map(cmd_vel.angular.z * 1000, -3000, 3000, 1000, 2000);
  rudder.write(angular);
  if (linear > 1){
    analogWrite(MOTOR_PIN_1, linear);
    analogWrite(MOTOR_PIN_2, 0);
    state_pos[0] = state_pos[0]+getEncoderCount(ENCODER_LEFT, 0);
    state_pos[1] = state_pos[1]+getEncoderCount(ENCODER_RIGHT, 1);
  }
  else if (linear < -1){
    analogWrite(MOTOR_PIN_2, -linear);
    analogWrite(MOTOR_PIN_1, 0);
    state_pos[0] = state_pos[0]-getEncoderCount(ENCODER_LEFT, 0);
    state_pos[1] = state_pos[1]-getEncoderCount(ENCODER_RIGHT, 1);
  }
  else if (linear == 0){
    analogWrite(MOTOR_PIN_2, 0);
    analogWrite(MOTOR_PIN_1, 0);
    }
  }

sensor_msgs::JointState state_msg;

ros::Publisher state_pub("joint_states", &state_msg);
ros::Subscriber<geometry_msgs::Twist> drive_sub("cmd_vel", drive_cb);

void setup() {
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  rudder.attach(RUDDER_PIN);
  rudder.write(1500);

  //nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(drive_sub);
  nh.advertise(state_pub);

  state_msg.header.frame_id =  "/driver_states";
  state_msg.name_length = NUM_JOINTS;
  state_msg.velocity_length = NUM_JOINTS;
  state_msg.position_length = NUM_JOINTS;
  state_msg.effort_length = NUM_JOINTS;
  state_msg.name = state_names;
  state_msg.position = state_pos;
  state_msg.velocity = state_vel;
  state_msg.effort = state_eff;
  }

void loop(){ 
  if((millis() - last_ms) >= RATE_MS){
    last_ms = millis();
    state_pos[2] = angular; 
    state_msg.header.stamp = nh.now();
    state_pub.publish(&state_msg);
    state_pos[0] = 0;
    state_pos[1] = 0;
  }
  nh.spinOnce();
  //delay(5);
}

int getEncoderCount(int encoder, int side)
{
  int count = 0;
  if (digitalRead(encoder)==HIGH)
    {
    encodersFlag[side]=HIGH;
    }
  if (digitalRead(encoder)==LOW && encodersFlag[side]==HIGH)
    {
    count++;
    encodersFlag[side]=LOW;
    }
}
