#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "Battery.h"

// Private Methods //////////////////////////////////////////////////////////////

float Battery::analog2voltage(int bat_pin, int cell) {
    float analog = analogRead(bat_pin);                             //читаем значение и преобразем в вольты
    float count = 1.0;                                              //счетчик количества элементов для расчета среднего
    if (_cell_voltage[cell] > 0.0)                                  //если значение в массиве уже есть, то значит нужно выводить среднее (элементов 2)
    {
        count = 2.0;
    }

    if (analog > 10)                                                //устанавливаем порог напряжения (фильтр шумов на входе)
    {
        float cell_volt = (_kA * analog) + _kB;                     //линейная функция тренда для расчета вольтажа
        return (_cell_voltage[cell] + cell_volt)  / count;          //замеряем и записываем, затем получим среднее
    }
    else
    {
        return 0.0;
    }
}

// Public Methods //////////////////////////////////////////////////////////////

Battery::Battery(
        const char *topic_name,
        const uint8_t *bat_pins,
        uint8_t bat_cells,
        float cell_voltage_max,
        float cell_voltage_min,
        float kA,
        float kB)
    : _bat_pins(bat_pins),
      _bat_cells(bat_cells),
      _cell_voltage_max(cell_voltage_max),
      _cell_voltage_min(cell_voltage_min),
      _kA(kA),
      _kB(kB),
      _msg(),
      _pub(topic_name, &_msg)
{

    _cell_voltage = (float *) malloc(_bat_cells * sizeof(float));
    for(int i = 0; i < bat_cells; i++) {
        pinMode(bat_pins[i], INPUT_ANALOG);
    }
    resetCellVoltage();

    _msg.header.frame_id =  "/battery";    //заголовок
    _msg.cell_voltage_length = _bat_cells; //колличество банок (элементов массива)
    _msg.cell_voltage = _cell_voltage;     //массив значений напряжений на банках аккумулятора
    _msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    _msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    _msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
}

void Battery::init(ros::NodeHandle &nh) {
    _nh = &nh;
    _nh->advertise(_pub);
}

void Battery::publicBatteryInfo() {
    //Читаем и записываем значения напряжений в массив
    for(int i = 0; i < _bat_cells; i++) {
        _cell_voltage[i] = analog2voltage(_bat_pins[i], i);
    }

    // вычисляем напряжение 2-4 секций аккумулятора
    for(int i = _bat_cells - 1; i > 0; i--) {
        _cell_voltage[i] -= _cell_voltage[i-1];
    }

    // вычисляем суммарное напряжение
    _msg.voltage = 0;
    for(int i = 0; i < _bat_cells; i++) {
        _msg.voltage += _cell_voltage[i];
    }

    _msg.percentage = (_msg.voltage - _cell_voltage_min*_bat_cells) / (_cell_voltage_max*_bat_cells - _cell_voltage_min*_bat_cells); // считаем количество процентов возможного ресурса батареи
    if (_msg.percentage < 0) {
        _msg.percentage = 0.0;
    }
    if (_msg.percentage > 1) {
        _msg.percentage = 1.0;
    }

    _msg.header.stamp = _nh->now();     //фиксируем время сообщения
    _msg.present = _msg.voltage > 0;
    _pub.publish(&_msg);                //публикуем сообщение в топик

    //обнуляем данные секций
    resetCellVoltage();
}

void Battery::resetCellVoltage() {
    for(int i=0; i< _bat_cells; i++)
    {
        _cell_voltage[i] = 0.0;
    }
}
