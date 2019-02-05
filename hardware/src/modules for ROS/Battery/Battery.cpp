#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "Battery.h"

// Private Methods //////////////////////////////////////////////////////////////

float Battery::analog2voltage(int bat_pin, int cell) {
    float analog = analogRead(bat_pin);                     //читаем значение и преобразем в вольты
    float count = 1.0;                                      //счетчик количества элементов для расчета среднего
    if (cell_voltage[cell] > 0.0)                           //если значение в массиве уже есть, то значит нужно выводить среднее (элементов 2)
    {
        count = 2.0;
    }

    if (analog > 10)                                        //устанавливаем порог напряжения (фильтр шумов на входе)
    {
        float cell_volt = 0.0088316807*analog+0.031247058;    //линейная функция тренда для расчета вольтажа
        return (cell_voltage[cell] + cell_volt)  / count;     //замеряем и записываем, затем получим среднее
    }
    else
    {
        return 0.0;
    }
}

// Public Methods //////////////////////////////////////////////////////////////

Battery::Battery(uint8_t bat_balance_pin_1, uint8_t bat_balance_pin_2, uint8_t bat_balance_pin_3, uint8_t bat_balance_pin_4, float bat_voltage_max, float bat_voltage_min, int cell_bat)
    : bat_balance_pin_1(bat_balance_pin_1),
      bat_balance_pin_2(bat_balance_pin_2),
      bat_balance_pin_3(bat_balance_pin_3),
      bat_balance_pin_4(bat_balance_pin_4),
      bat_voltage_max(bat_voltage_max),
      bat_voltage_min(bat_voltage_min),
      cell_bat(cell_bat)
{
    pinMode(bat_balance_pin_1, INPUT_ANALOG);
    pinMode(bat_balance_pin_2, INPUT_ANALOG);
    pinMode(bat_balance_pin_3, INPUT_ANALOG);
    pinMode(bat_balance_pin_4, INPUT_ANALOG);

    resetCellVoltage();

    battery_msg.header.frame_id =  "/battery";  //заголовок
    battery_msg.cell_voltage_length = cell_bat; //колличество банок (элементов массива)
    battery_msg.cell_voltage = cell_voltage;    //массив значений напряжений на банках аккумулятора
    battery_msg.present = true;                 //батарея присутствует
    battery_msg.power_supply_technology = 3;    //тип батареи LiPo
    battery_msg.power_supply_status = 2;        //разряжается
    battery_msg.power_supply_health = 0;        //неизвесно
    battery_pub.setInit("aaa", &battery_msg);
    //battery_pub = bat; //ros::Publisher();
}

void Battery::init(ros::NodeHandle &nh) {
    this->nh = nh;
    //battery_pub.setTopicName("battery");
    //battery_pub.setMsg(&battery_msg);
    //battery_pub.setEndpointType();
    this->nh.advertise(battery_pub);
}

void Battery::publicBatteryInfo() {
    //Читаем и записываем значения напряжений в массив
    cell_voltage[0] = analog2voltage(bat_balance_pin_1, 0);
    cell_voltage[1] = analog2voltage(bat_balance_pin_2, 1);
    cell_voltage[2] = analog2voltage(bat_balance_pin_3, 2);
    cell_voltage[3] = analog2voltage(bat_balance_pin_4, 3);

    // вычисляем напряжение 2-4 секций аккумулятора
    for(int i=cell_bat-1;i>0;i--)
    {
        cell_voltage[i] -= cell_voltage[i-1];
    }

    battery_msg.voltage = cell_voltage[0] + cell_voltage[1] + cell_voltage[2] + cell_voltage[3];            // расчет общего напряжения аккумулятора
    battery_msg.percentage = (battery_msg.voltage - BAT_VOLTAGE_MIN) / (BAT_VOLTAGE_MAX - BAT_VOLTAGE_MIN); // считаем количество процентов возможного ресурса батареи
    if (battery_msg.percentage < 0) {
        battery_msg.percentage = 0.0;
    }

    battery_msg.header.stamp = nh.now();     //фиксируем время сообщения

    nh.logwarn("pub");
    battery_pub.publish(&battery_msg);       //публикуем сообщение в топик

    //обнуляем данные секций
    resetCellVoltage();
}

void Battery::resetCellVoltage() {
    for(int i=0;i<cell_bat;i++)
    {
        cell_voltage[i] = 0.0;
    }
}
