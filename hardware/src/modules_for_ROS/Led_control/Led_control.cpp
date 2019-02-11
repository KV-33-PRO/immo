#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "Led_control.h"

// Private Methods //////////////////////////////////////////////////////////////

void LedControl::head_cb(const std_msgs::UInt8& mode) {

}

void LedControl::turn_cb(const std_msgs::UInt8& mode) {

}

void LedControl::back_cb(const std_msgs::UInt8& mode) {

}

void LedControl::brake_cb(const std_msgs::UInt8& mode) {

}

void LedControl::flasher_cb(const std_msgs::UInt8& mode) {

}

void LedControl::position_cb(const std_msgs::UInt8& mode) {

}

// Public Methods //////////////////////////////////////////////////////////////

LedControl::LedControl() :
    _head_sub("light_control/head", LedControl::head_cb),
    _turn_sub("light_control/turn", LedControl::turn_cb),
    _back_sub("light_control/back", LedControl::back_cb),
    _brake_sub("light_control/brake", LedControl::brake_cb),
    _flasher_sub("light_control/flasher", LedControl::flasher_cb),
    _position_sub("light_control/position", LedControl::position_cb)
{

}

void LedControl::init(ros::NodeHandle &nh) {
        _nh = &nh;
        _nh->subscribe(_head_sub);
        _nh->subscribe(_turn_sub);
        _nh->subscribe(_back_sub);
        _nh->subscribe(_brake_sub);
        _nh->subscribe(_flasher_sub);
        _nh->subscribe(_position_sub);
}

void LedControl::led(){

}
