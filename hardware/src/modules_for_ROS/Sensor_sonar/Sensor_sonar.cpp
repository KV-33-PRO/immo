#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "Sensor_sonar.h"

// Private Methods //////////////////////////////////////////////////////////////

// Public Methods //////////////////////////////////////////////////////////////

Sensor_sonar::Sensor_sonar(
        const char *topic_name,
        uint8_t trig_pin,
        uint8_t echo_pin,
        float range_min,
        float range_max)
    : _trig_pin(trig_pin),
      _echo_pin(echo_pin),
      _msg(),
      _pub(topic_name, &_msg),
      _sonar(trig_pin, echo_pin),
      _kf(2, 2, 0.01)
{

    _msg.header.frame_id = topic_name;
    _msg.field_of_view = 0.10000000149;
    _msg.min_range = range_min;
    _msg.max_range = range_max;
}

void Sensor_sonar::init(ros::NodeHandle &nh) {
    _nh = &nh;
    _nh->advertise(_pub);
}

void Sensor_sonar::publicSensorInfo() {
    _msg.range = _kf.updateEstimate(_sonar.read()/100.0);
    _msg.header.stamp = _nh->now();     //фиксируем время сообщения
    _pub.publish(&_msg);                //публикуем сообщение в топик
}
