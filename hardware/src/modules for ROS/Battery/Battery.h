#ifndef Battery_h
#define Battery_h
#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define BAT_VOLTAGE_MAX           16.4             // максимальный вольтаж батареи (для расчета процентов)
#define BAT_VOLTAGE_MIN           14.0             // минимальный вольтаж батареи (для расчета процентов)
#define BAT_CELL                  4                // количество банок аккумулятора

class Battery {
private:

    uint8_t bat_balance_pin_1;
    uint8_t bat_balance_pin_2;
    uint8_t bat_balance_pin_3;
    uint8_t bat_balance_pin_4;

    float bat_voltage_max;
    float bat_voltage_min;
    int cell_bat;

    float cell_voltage[BAT_CELL];

    sensor_msgs::BatteryState battery_msg;
    ros::Publisher battery_pub;
    ros::NodeHandle nh;

    float analog2voltage(int bat_pin, int cell);
    void resetCellVoltage();
	
public:

    Battery(uint8_t bat_balance_pin_1, uint8_t bat_balance_pin_2, uint8_t bat_balance_pin_3, uint8_t bat_balance_pin_4, float bat_voltage_max = BAT_VOLTAGE_MAX, float bat_voltage_min = BAT_VOLTAGE_MIN, int cell_bat = BAT_CELL);
    void init(ros::NodeHandle& nh);
    void publicBatteryInfo();
	
};

#endif
