#include <ros.h>
#include <Led_control.h>

#define RATE_MS                   20       // задержка для публикации в топик

unsigned long last_ms;

ros::NodeHandle nh;


void setup() {
  nh.initNode();
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик

  }
  nh.spinOnce();
}
