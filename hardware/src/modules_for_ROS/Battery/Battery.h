#ifndef Battery_h
#define Battery_h
#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

#define CELL_VOLTAGE_MAX           4.2            // максимальный вольтаж ячейки батареи (для расчета процентов)
#define CELL_VOLTAGE_MIN           3.5            // минимальный вольтаж ячейки батареи (для расчета процентов)

class Battery {
public:
    Battery(const char *topic_name,
            const uint8_t *bat_pins,
            uint8_t bat_cells,
            float cell_voltage_max = CELL_VOLTAGE_MAX,
            float cell_voltage_min = CELL_VOLTAGE_MIN,
            float kA = 0.0088316807,
            float kB = 0.031247058);
    void init(ros::NodeHandle& nh);
    void publicBatteryInfo();
private:
    const uint8_t *_bat_pins;
    uint8_t _bat_cells;
    float _cell_voltage_max;
    float _cell_voltage_min;
    float _kA;
    float _kB;
    sensor_msgs::BatteryState _msg;
    ros::Publisher _pub;
    float *_cell_voltage;
    ros::NodeHandle *_nh;
    float analog2voltage(int bat_pin, int cell);
    void resetCellVoltage();
};

#endif
