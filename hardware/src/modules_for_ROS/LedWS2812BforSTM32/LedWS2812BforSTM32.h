#ifndef LedWS2812BforSTM32_h
#define LedWS2812BforSTM32_h
#include <ros.h>
#include <WS2812B.h>


#define NUM_LEDS                  20
#define COUNT_LEDS_HEADLIGHT      5
#define COUNT_INDICATION_STATUS   8
#define RATE_TURN_MS              100
#define RATE_FLASHER_MS           50

//SIDE
#define FRONT 0
#define REAR  1
#define RIGHT 2
#define LEFT  3
#define ALL   4

//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

class LedWS2812BforSTM32 {
public:
    LedWS2812BforSTM32();
    void indication(bool *indication_status);
    void init();
    String getData();
private:
    void position();
    void head();
    void turn();
    void turn(int mode);
    void backAndBrake();
    void flaser();
    void setSideColorLeds(int side, int led_start, int led_end, uint32_t color);
    void show();

    int _leds[NUM_LEDS/COUNT_LEDS_HEADLIGHT][COUNT_LEDS_HEADLIGHT]={{4, 3, 2, 1, 0},         //0 - левая передная
                                                                    {5, 6, 7, 8, 9},         //1 - правая передняя
                                                                    {14 ,13 ,12, 11, 10},    //2 - левая задняя
                                                                    {15, 16, 17, 18, 19}};  //3 - правая задняя*/
    uint32_t _leds_color[NUM_LEDS/COUNT_LEDS_HEADLIGHT][COUNT_LEDS_HEADLIGHT];
    bool _indication_status[COUNT_INDICATION_STATUS];

    WS2812B _strip = WS2812B(NUM_LEDS);
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
    unsigned long _turn_last_time;

    bool _brake_on;
    unsigned long _brake_last_time;

    int _flasher_side=LEFT;
    int _flasher_count=0;
    bool _flasher_status=false;
    unsigned long _flasher_last_time;

};

extern LedWS2812BforSTM32 Led;

#endif
