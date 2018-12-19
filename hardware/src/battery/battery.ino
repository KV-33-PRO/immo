#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define RATE_MS                   20       // задержка для публикации в топик

#define BAT_BALANCE_PIN_1               PA0      // вход для балансового порта батареи 1
#define BAT_BALANCE_PIN_2               PA1      // вход для балансового порта батареи 2
#define BAT_BALANCE_PIN_3               PA2      // вход для балансового порта батареи 3
#define BAT_BALANCE_PIN_4               PA3      // вход для балансового порта батареи 4

#define BAT_ANALOG_1V           112.783      // экспериментально полученное значение 1 вольта (аналоговый порт)
#define BAT_VOLTAGE_MAX         16.4         // максимальный вольтаж батареи (для расчета процентов)
#define BAT_VOLTAGE_MIN         14.0         // минимальный вольтаж батареи (для расчета процентов)
#define CELL_BAT                4            // количество банок аккумулятора

float cell_voltage[CELL_BAT] = {0, 0, 0, 0};
unsigned long last_ms;

ros::NodeHandle nh;
sensor_msgs::BatteryState battery_msg;

ros::Publisher battery_pub("battery", &battery_msg);                    // инициализация издателя топика "battery"

void setup() {
  pinMode(BAT_BALANCE_PIN_1, INPUT_ANALOG);
  pinMode(BAT_BALANCE_PIN_2, INPUT_ANALOG);
  pinMode(BAT_BALANCE_PIN_3, INPUT_ANALOG);
  pinMode(BAT_BALANCE_PIN_4, INPUT_ANALOG);

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(battery_pub);

  battery_msg.header.frame_id =  "/battery";  // заголовок
  battery_msg.cell_voltage_length = CELL_BAT; // колличество банок (элементов массива)
  battery_msg.cell_voltage = cell_voltage;    // массив значений напряжений на банках аккумулятора
  battery_msg.present = true;                 // батарея присутствует
  battery_msg.power_supply_technology = 3;    // тип батареи LiPo
  battery_msg.power_supply_status = 2;        // разряжается
  battery_msg.power_supply_health = 0;        // неизвесно
}

void loop() {
  //Читаем и записываем значения напряжений в массив
  cell_voltage[0] = analog2voltage(BAT_BALANCE_PIN_1, 0);
  cell_voltage[1] = analog2voltage(BAT_BALANCE_PIN_2, 1);
  cell_voltage[2] = analog2voltage(BAT_BALANCE_PIN_3, 2);
  cell_voltage[3] = analog2voltage(BAT_BALANCE_PIN_4, 3);

    cell_voltage[0] = 4.0;
    cell_voltage[1] = 8.0;
    cell_voltage[2] = 12.0;
    cell_voltage[3] = 16.0;

  if (millis() - last_ms >= RATE_MS) {     // публикуем не чаще чем RATE_MS
    last_ms = millis();   // фиксируем последнее время публикации сообщения в топик

    cell_voltage[1] -= cell_voltage[0];                    // вычисляем напряжение на банке 2
    cell_voltage[2] -= cell_voltage[0]+cell_voltage[1];                // вычисляем напряжение на банке 3
    cell_voltage[3] -= cell_voltage[0]+cell_voltage[1]+cell_voltage[2];           // вычисляем напряжение на банке 4

    battery_msg.voltage = cell_voltage[0] + cell_voltage[1] + cell_voltage[2] + cell_voltage[3];            // расчет общего напряжения аккумулятора
    battery_msg.percentage = (battery_msg.voltage - BAT_VOLTAGE_MIN) / (BAT_VOLTAGE_MAX - BAT_VOLTAGE_MIN); // считаем количество процентов возможного ресурса батареи
    if (battery_msg.percentage < 0) {
      battery_msg.percentage = 0.0;
    }
    
    battery_msg.header.stamp = nh.now();     // фиксируем время сообщения
    battery_pub.publish(&battery_msg);      // публикуем сообщение в топик
    //обнуляем данные
    cell_voltage[0] = 0.0;
    cell_voltage[1] = 0.0;
    cell_voltage[2] = 0.0;
    cell_voltage[3] = 0.0;
  }
  nh.spinOnce();
}

float analog2voltage(int bat_pin, int cell) {
  float volt = analogRead(bat_pin) / BAT_ANALOG_1V;          // читаем значение и преобразем в вольты
  float count = 1.0;                                      // счетчик количества элементов для расчета среднего
  if (cell_voltage[cell] > 0.0)                           // если значение в массиве уже есть, то значит нужно выводить среднее (элементов 2)
  {
    count = 2.0;
  }
  if (volt > 0.5)                                        // устанавливаем порог напряжения (фильтр шумов на входе) 
  {
    return (cell_voltage[cell] + volt) / count;     // замеряем и записываем, затем получим среднее
  }
  else
  {
    return 0.0;
  }
  
}
