#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <Battery.h>

#define RATE_MS                   20       // задержка для публикации в топик

#define BAT_BALANCE_PIN_1         PA0      // вход для балансового порта батареи 1
#define BAT_BALANCE_PIN_2         PA1      // вход для балансового порта батареи 2
#define BAT_BALANCE_PIN_3         PA2      // вход для балансового порта батареи 3
#define BAT_BALANCE_PIN_4         PA3      // вход для балансового порта батареи 4


unsigned long last_ms;

ros::NodeHandle nh;

uint8_t battery_pins[4] = {
    BAT_BALANCE_PIN_1,
    BAT_BALANCE_PIN_2,
    BAT_BALANCE_PIN_3,
    BAT_BALANCE_PIN_4
};

Battery battery("/battery", battery_pins, sizeof(battery_pins) / sizeof(uint8_t));

void setup() {
  nh.initNode();
  battery.init(nh);
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();                    //фиксируем последнее время публикации сообщения в топик
    battery.publicBatteryInfo();
  }
  nh.spinOnce();
}
