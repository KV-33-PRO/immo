#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#define RATE_MS                 20    // задержка для публикации

#define RUDDER_PIN              11    // подключение сервопривода
#define GRAD_RUDDER             65.0  // угол поворота колес между положениями "влево" - "вправо" (в градусах)
#define COUNT_ENCODER_WHEEL     3000  // число импульсов на один оборот колеса

#define MOTOR_PIN_1             5     // двигатель +/-
#define MOTOR_PIN_2             6     // двигатель -/+
#define ENCODER_LEFT            2     // прерывание на пине 2
#define ENCODER_RIGHT           3     // прерывание на пине 3
#define DIRECTION_MOTOR_LEFT    7     // выход с левого мотора (для направления движения)
#define DIRECTION_MOTOR_RIGHT   8     // выход с правого мотора (для направления движения) 

#define NUM_JOINTS              3

char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel", "rudder"};
float state_pos[NUM_JOINTS] = {0, 0, 0};
float state_vel[NUM_JOINTS] = {0, 0, 0};  //rad/s
float state_eff[NUM_JOINTS] = {0, 0, 0};
unsigned long last_ms;

ros::NodeHandle nh;

Servo rudder;
float linear;
float angular;

void drive_cb(const geometry_msgs::Twist& cmd_vel) {
  linear = map(cmd_vel.linear.x * 1000, -1000, 1000, -255, 255);
  angular = map(cmd_vel.angular.z * 1000, -3000, 3000, 1000, 2000);
  rudder.write(angular);
  if (linear > 1) {
    analogWrite(MOTOR_PIN_1, linear);
    analogWrite(MOTOR_PIN_2, 0);
  }
  else if (linear < -1) {
    analogWrite(MOTOR_PIN_2, -linear);
    analogWrite(MOTOR_PIN_1, 0);
  }
  else if (linear == 0) {
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
  pinMode(DIRECTION_MOTOR_LEFT, INPUT);
  pinMode(DIRECTION_MOTOR_RIGHT, INPUT);
  rudder.attach(RUDDER_PIN);
  rudder.write(1500);
  attachInterrupt(0, doEncoderLeft, FALLING);  //pin2
  attachInterrupt(1, doEncoderRight, FALLING); //pin3

  nh.getHardware()->setBaud(500000);
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

void loop() {
    unsigned long t = millis() - last_ms;
  if ((t) >= RATE_MS) {
    last_ms = millis();

    state_vel[0] = ((((state_vel[0] / (COUNT_ENCODER_WHEEL / 360)) * M_PI) / 180) / t) * (1000/t);   //преобразование импульсы > градусы > радианы > рад/с
    state_vel[1] = ((((state_vel[1] / (COUNT_ENCODER_WHEEL / 360)) * M_PI) / 180) / t) * (1000/t);   //преобразование импульсы > градусы > радианы > рад/с
    state_pos[2] = -((angular - 1500) * (GRAD_RUDDER / 1000)) * M_PI / 180; //преобразование значения с сервы(1000..2000) > (-500..500) > градусы > радианы

    state_msg.header.stamp = nh.now();
    state_pub.publish(&state_msg);
    state_pos[2] = 0;
  }
  nh.spinOnce();
}

// Interrupt on B changing state
void doEncoderLeft() {
  state_vel[0] += getEncoderCount(DIRECTION_MOTOR_LEFT, 0);
}
void doEncoderRight() {
  state_vel[1] += getEncoderCount(DIRECTION_MOTOR_RIGHT, 1);
}

int getEncoderCount(int direction_motor, int side)
{
  int count = 0;
  if (digitalRead(direction_motor) == HIGH & side == 0)
  {
    count++;
  }
  else
  {
    count--;
  }
  if (digitalRead(direction_motor) == HIGH & side == 1)
  {
    count--;
  }
  else
  {
    count++;
  }
  return count;
}
