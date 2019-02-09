#include <ros.h>
#include <Sensor_sonar.h>

#define RATE_MS                   20       // задержка для публикации в топик

#define SONAR_LEFT_TRIG_PIN       PA15
#define SONAR_LEFT_ECHO_PIN       PB3
#define SONAR_RIGHT_TRIG_PIN      PB4
#define SONAR_RIGHT_ECHO_PIN      PB5

#define SONAR_RANGE_MIN           0.3
#define SONAR_RANGE_MAX           3.0

unsigned long last_ms;

ros::NodeHandle nh;

Sensor_sonar sonar_left("/range/front/sonar_left", SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_RANGE_MIN, SONAR_RANGE_MAX);
Sensor_sonar sonar_right("/range/front/sonar_right", SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN, SONAR_RANGE_MIN, SONAR_RANGE_MAX);

void setup() {
  nh.initNode();
  sonar_left.init(nh);
  sonar_right.init(nh);
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
    sonar_left.publicSensorInfo();
    sonar_right.publicSensorInfo();
  }
  nh.spinOnce();
}
