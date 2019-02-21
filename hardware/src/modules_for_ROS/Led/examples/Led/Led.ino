#include <ros.h>
#include <Led.h>

#define RATE_MS                   20       // задержка для публикации в топик

unsigned long last_ms;

ros::NodeHandle nh;
Led led;

void setup() {
  nh.initNode();
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
    led.position(1);
  }
  nh.spinOnce();
}
