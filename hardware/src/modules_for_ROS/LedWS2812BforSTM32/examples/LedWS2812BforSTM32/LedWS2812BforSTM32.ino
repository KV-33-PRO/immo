#include <ros.h>
#include <LedWS2812BforSTM32.h>

#define RATE_MS                   20       // задержка для публикации в топик

unsigned long last_ms;

ros::NodeHandle nh;
bool indication_status[8] =
     {false,   //габариты
      false,   //ближний свет
      false,   //дальний свет
      false,   //левый поротник
      false,   //правый поворотник
      false,   //стоп сигнал
      false,   //задний ход
      false};  //мигалка

void setup() {
  nh.initNode();
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
    Led.indication(indication_status);
  }
  nh.spinOnce();
}
