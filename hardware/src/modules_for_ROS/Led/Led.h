#ifndef Led_h
#define Led_h
#include <ros.h>
#include <WS2812B.h>

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



#define NUM_LEDS    20
#define COUNT_LEDS_HEADLIGHT    5
//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

class Led {
public:
    Led();
    void head(uint8_t mode);
    void turn(uint8_t mode);
    void back(uint8_t mode);
    void brake(uint8_t mode);
    void flaser(uint8_t mode);
    void position(uint8_t mode);
private:
    void stripShowColor(int count_lamps, int lamp1, int lamp2, int lamp3, int lamp4, uint32_t color);
    void Led::setColorLeds(int side, int id_led, uint32_t color);
    int *_leds;
    WS2812B _strip;
    uint32_t _yelloy;
    uint32_t _blue;
    uint32_t _black;
    uint32_t _red_high;
    uint32_t _red_low;
    uint32_t _white_high;
    uint32_t _white_middle;
    uint32_t _white_low;
    unsigned long time_break_last;
};

#endif
