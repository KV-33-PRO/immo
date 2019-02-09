#ifndef Sensor_sonar_h
#define Sensor_sonar_h
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <Ultrasonic.h>
#include <SimpleKalmanFilter.h>

class Sensor_sonar {
public:
    Sensor_sonar(const char *topic_name,
            uint8_t _trig_pin,
            uint8_t _echo_pin,
            float range_min,
            float range_max);
    void init(ros::NodeHandle& nh);
    void publicSensorInfo();
private:
    uint8_t _trig_pin;
    uint8_t _echo_pin;
    sensor_msgs::Range _msg;
    ros::Publisher _pub;
    ros::NodeHandle *_nh;
    Ultrasonic _sonar;
    SimpleKalmanFilter _kf;
};

#endif
