#include <ros.h>
#include <LedControl.h>

#define RATE_MS                   20       // задержка для публикации в топик

unsigned long last_ms;

ros::NodeHandle nh;
LedControl led;

void setup() {
  nh.initNode();
  led.init(nh);
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
    led.disco();
  }
  nh.spinOnce();
}
