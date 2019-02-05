#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <Battery.h>

#define RATE_MS                   20       // задержка для публикации в топик

#define BAT_BALANCE_PIN_1         PA0      // вход для балансового порта батареи 1
#define BAT_BALANCE_PIN_2         PA1      // вход для балансового порта батареи 2
#define BAT_BALANCE_PIN_3         PA2      // вход для балансового порта батареи 3
#define BAT_BALANCE_PIN_4         PA3      // вход для балансового порта батареи 4

#define BAT_VOLTAGE_MAX           16.4             // максимальный вольтаж батареи (для расчета процентов)
#define BAT_VOLTAGE_MIN           14.0             // минимальный вольтаж батареи (для расчета процентов)
#define CELL_BAT                  4                // количество банок аккумулятора

unsigned long last_ms;

ros::NodeHandle nh;

Battery battery(BAT_BALANCE_PIN_1, BAT_BALANCE_PIN_2, BAT_BALANCE_PIN_3, BAT_BALANCE_PIN_4, BAT_VOLTAGE_MAX, BAT_VOLTAGE_MIN, CELL_BAT);

void setup() {
  nh.initNode();
  battery.init(nh);
}

void loop() {
  if (millis() - last_ms >= RATE_MS) {     //публикуем не чаще чем RATE_MS
    last_ms = millis();   //фиксируем последнее время публикации сообщения в топик
    battery.publicBatteryInfo();
  }
  nh.spinOnce();
}
