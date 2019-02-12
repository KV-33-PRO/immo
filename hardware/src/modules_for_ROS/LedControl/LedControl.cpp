#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "LedControl.h"

//TOPICS
#define HEAD 0
#define TURN 1
#define BACK 2
#define BRAKE 3
#define FLASHER 4
#define POSITION 5

//TURN
#define TURN_OFF   0
#define TURN_LEFT  1
#define TURN_RIGHT 2
#define TURN_FLASH 3

//HEAD
#define HEAD_OFF         0
#define HEAD_DIPPED_BEAM 1
#define HEAD_HIGH_BEAM   2

//BACK
#define BACK_OFF 0
#define BACK_ON  1

//BRAKE
#define BRAKE_OFF 0
#define BRAKE_ON  1

//FLASHER
#define FLASHER_OFF 0
#define FLASHER_ON  1

//POSITION_LAMPS
#define POSITION_LAMPS_OFF 0
#define POSITION_LAMPS_ON  1


// Private Methods //////////////////////////////////////////////////////////////

void LedControl::head_cb(const std_msgs::UInt8& mode) {
    _indication[HEAD][0]=mode;
    if(statusChanged(HEAD)){
        switch (_indication[HEAD][0].data) {
        case HEAD_OFF:
            //отключить
            break;
        case HEAD_DIPPED_BEAM:
            //включить ближний
            break;
        case HEAD_HIGH_BEAM:
            //включить дальний
            break;
        default:
            break;
        }
    }
}

void LedControl::turn_cb(const std_msgs::UInt8& mode) {
    _indication[TURN][0]=mode;
    if(statusChanged(TURN)){
        switch (_indication[TURN][0].data) {
        case TURN_OFF:
            //отключить
            break;
        case TURN_LEFT:
            //включить левый поворотник
            break;
        case TURN_RIGHT:
            //включить правый поворотник
            break;
        case TURN_FLASH:
            //включить аварийный сигнал
            break;
        default:
            break;
        }
    }
}

void LedControl::back_cb(const std_msgs::UInt8& mode) {
    _indication[BACK][0]=mode;
    if(statusChanged(BACK)){
        switch (_indication[BACK][0].data) {
        case BACK_OFF:
            //отключить
            break;
        case BACK_ON:
            //включить
            break;
        default:
            break;
        }
    }
}

void LedControl::brake_cb(const std_msgs::UInt8& mode) {
    _indication[BRAKE][0]=mode;
    if(statusChanged(BRAKE)){
        switch (_indication[BRAKE][0].data) {
        case BRAKE_OFF:
            //отключить
            break;
        case BRAKE_ON:
            //включить
            break;
        default:
            break;
        }
    }
}

void LedControl::flasher_cb(const std_msgs::UInt8& mode) {
    _indication[FLASHER][0]=mode;
    if(statusChanged(FLASHER)){
        switch (_indication[FLASHER][0].data) {
        case FLASHER_OFF:
            //отключить
            break;
        case FLASHER_ON:
            //включить
            break;
        default:
            break;
        }
    }
}

void LedControl::position_cb(const std_msgs::UInt8& mode) {
    _indication[POSITION][0]=mode;
    if(statusChanged(POSITION)){
        switch (_indication[POSITION][0].data) {
        case POSITION_LAMPS_OFF:
            //отключить
            break;
        case POSITION_LAMPS_ON:
            //включить
            break;
        default:
            break;
        }
    }
}

bool LedControl::statusChanged(int strIndication){
    if(_indication[strIndication][0].data!=_indication[strIndication][1].data)
    {
        _indication[strIndication][1]=_indication[strIndication][0];
        return true;
    }
    else
    {
        return false;
    }
}

// Public Methods //////////////////////////////////////////////////////////////

LedControl::LedControl() :
    _head_sub("light_control/head", &LedControl::head_cb, this),
    _turn_sub("light_control/turn", &LedControl::turn_cb, this),
    _back_sub("light_control/back", &LedControl::back_cb, this),
    _brake_sub("light_control/brake", &LedControl::brake_cb, this),
    _flasher_sub("light_control/flasher", &LedControl::flasher_cb, this),
    _position_sub("light_control/position", &LedControl::position_cb, this)
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
