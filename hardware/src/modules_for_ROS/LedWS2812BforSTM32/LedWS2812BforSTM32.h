#ifndef LedWS2812BforSTM32_h
#define LedWS2812BforSTM32_h
#include <ros.h>
#include <WS2812B.h>


#define NUM_LEDS                  20
#define COUNT_LEDS_HEADLIGHT      5
#define COUNT_INDICATION_STATUS   8
#define RATE_TURN_MS              50
//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

class LedWS2812BforSTM32 {
public:
    LedWS2812BforSTM32();
    void indication(bool *indication_status);
private:
    void position();
    void head();
    void turn();
    void turn(int mode);
    void backAndBrake();
    void flaser();
    void setSideColorLeds(int side, int led_start, int led_end, uint32_t color);
    void show();

    int _leds[4][COUNT_LEDS_HEADLIGHT];
    uint32_t _leds_color[4][COUNT_LEDS_HEADLIGHT];
    bool _indication_status[COUNT_INDICATION_STATUS];

    WS2812B _strip;
    uint32_t _yelloy;
    uint32_t _blue;
    uint32_t _black;
    uint32_t _red_high;
    uint32_t _red_low;
    uint32_t _white_high;
    uint32_t _white_middle;
    uint32_t _white_low;

    int _turn_count;
    int _turn_status;
    unsigned long _time_turn_last;
    unsigned long _time_brake_last;
    bool _brake_on;
};

extern LedWS2812BforSTM32 Led;

#endif
