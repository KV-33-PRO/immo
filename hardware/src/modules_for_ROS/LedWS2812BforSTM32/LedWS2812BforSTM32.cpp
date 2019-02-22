#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "LedWS2812BforSTM32.h"

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

//LAMPS
#define FL 0
#define FR 1
#define RL 2
#define RR 3

//SIDE
#define FRONT 0
#define REAR  1
#define RIGHT 2
#define LEFT  3
#define ALL   4

//INDICATION STATUS
#define POSITION_LAMPS     0
#define DIPPED_BEAM_LAMPS  1
#define HIGH_BEAM_LAMPS    2
#define TURN_LEFT_LAMPS    3
#define TURN_RIGHT_LAMPS   4
#define BRAKE_LAMPS        5
#define BACK_LAMPS         6
#define FLASHER_LAMPS      7

// Private Methods //////////////////////////////////////////////////////////////

//Заполнить определенные диоды, определенным цветом, определенной стороны.
void LedWS2812BforSTM32::setSideColorLeds(int side, int led_start, int led_end, uint32_t color){
    for(int i = led_start; i<=led_end; i++)
    {
        switch(side)
        {
        case FRONT:
            _leds_color[FL][i] = color;
            _leds_color[FR][i] = color;
            break;
        case REAR:
            _leds_color[RL][i] = color;
            _leds_color[RR][i] = color;
            break;
        case RIGHT:
            _leds_color[FR][i] = color;
            _leds_color[RR][i] = color;
            break;
        case LEFT:
            _leds_color[FL][i] = color;
            _leds_color[RL][i] = color;
            break;
        case ALL:
            for(int lamps = 0; lamps<4; lamps++)
            _leds_color[lamps][i] = color;
            break;
        default:
            break;
        }
    }
}

void LedWS2812BforSTM32::position(){
    if(_indication_status[POSITION_LAMPS])
    {
        setSideColorLeds(FRONT, 0, 4, _white_low);
        setSideColorLeds(REAR, 0, 4, _red_low);
    }
    else
    {
        setSideColorLeds(ALL, 0, 4, _black);
    }
}

void LedWS2812BforSTM32::head(){
    if(_indication_status[DIPPED_BEAM_LAMPS])
    {
        setSideColorLeds(FRONT, 0, 4, _white_middle);
    }
    if(_indication_status[HIGH_BEAM_LAMPS])
    {
        setSideColorLeds(FRONT, 0, 4, _white_high);
    }
}

void LedWS2812BforSTM32::backAndBrake(){
    if(_indication_status[BACK_LAMPS] && _indication_status[BRAKE_LAMPS])
    {
        setSideColorLeds(REAR, 0, 1, _white_high);
        setSideColorLeds(REAR, 2, 4, _red_high);   //Просто включается (не моргает)
    }
    else
    {
        if(_indication_status[BACK_LAMPS])
        {
        setSideColorLeds(REAR, 0, 4, _white_high);
        }
        if(_indication_status[BACK_LAMPS])
        {
        setSideColorLeds(REAR, 0, 4, _red_high);  //Просто включается (не моргает)
        }
    }
}

void LedWS2812BforSTM32::turn(){
    if(_turn_status==TURN_OFF)
    {
        if(_indication_status[TURN_LEFT] && _indication_status[TURN_RIGHT])
        {
            _turn_status=TURN_FLASH;
        }
        else
        {
            if(_indication_status[TURN_LEFT])
            {
                _turn_status=TURN_LEFT;
            }
            if(_indication_status[TURN_RIGHT])
            {
                _turn_status=TURN_RIGHT;
            }
        }
    }
    turn(_turn_status);
}

void LedWS2812BforSTM32::turn(int mode){
    int side = 5;
    if(millis() - _time_turn_last >= RATE_TURN_MS)
    {
        if(_turn_count<5)
        {
            switch (mode) {
            case TURN_FLASH:
                side=ALL;
                break;
            case TURN_LEFT:
                side=LEFT;
                break;
            case TURN_RIGHT:
                side=RIGHT;
                break;
            default:
                break;
            }

            if(mode != TURN_OFF)
            {
            setSideColorLeds(side, 0, _turn_count, _yelloy);
            if(_turn_count<4)
            setSideColorLeds(side, _turn_count, 4, _black);
            }

            _turn_count++;
            _time_turn_last = millis();
        }
        else
        {
            _turn_count=0;
            _turn_status=TURN_OFF;
            setSideColorLeds(side, 0, 4, _black);
        }
    }
}

void LedWS2812BforSTM32::flaser(){
    if(_indication_status[POSITION_LAMPS])
    {
        //Надо мигать
        //    int wait = 50;
        //    for (int b = 0; b <= 5; b++) {
        //      for (int b = 0; b <= 3; b++) {
        //        stripShowColor(2, 1, 3, 0, 0, red);
        //        delay(wait);
        //        stripShowColor(4, 0, 1, 2, 3, black);
        //        delay(wait);
        //      }
        //      for (int b = 0; b <= 3; b++) {
        //        stripShowColor(2, 0, 2, 0, 0, blue);
        //        delay(wait);
        //        stripShowColor(4, 0, 1, 2, 3, black);
        //        delay(wait);
        //      }
        //    }
        //    stripShowColor(4, 0, 1, 2, 3, black);
        //    }
        //    else
        //    {
        //        stripShowColor(4, 0, 1, 2, 3, black);
        //    }
    }
}

void LedWS2812BforSTM32::show(){
    for(int lamps = 0; lamps<4; lamps++)
    {
        for(int led = 0; led<COUNT_LEDS_HEADLIGHT; led++){
            _strip.setPixelColor(_leds[lamps][led], _leds_color[lamps][led]);
        }
    }
    _strip.show();
}

// Public Methods //////////////////////////////////////////////////////////////

LedWS2812BforSTM32::LedWS2812BforSTM32()
    : _leds({{4, 3, 2, 1, 0},         //0 - левая передная
             {5, 6, 7, 8, 9},         //1 - правая передняя
             {14 ,13 ,12, 11, 10},    //2 - левая задняя
             {15, 16, 17, 18, 19}}),  //3 - правая задняя
      _strip(WS2812B(NUM_LEDS))
{
    _strip.begin();     // Sets up the SPI
    _strip.show();      // Clears the strip, as by default the strip data is set to all LED's off.
    //strip.setBrightness(8);

    _yelloy = _strip.Color(255, 128, 0);
    _blue = _strip.Color(0, 0, 255);
    _red_high = _strip.Color(255, 0, 0);
    _red_low = _strip.Color(100, 0, 0);
    _white_high = _strip.Color(255, 255, 255);
    _white_middle = _strip.Color(180, 180, 180);
    _white_low = _strip.Color(100, 100, 100);
    _black = _strip.Color(0, 0, 0);

    _turn_count = 0;
    _turn_status=TURN_OFF;

    for(int status = 0; status<COUNT_INDICATION_STATUS; status++){
        _indication_status[status] = false;
    }
    setSideColorLeds(ALL, 0, 4, _black);
}

void LedWS2812BforSTM32::indication(bool *indication_status)
{
    for(int status = 0; status<COUNT_INDICATION_STATUS; status++){
        _indication_status[status] = indication_status[status];
    }

    //запускаем по порядку заполение массива цветами (порядок приоритета)
    position();
    head();
    backAndBrake();
    turn();
    flaser();

    //Вывод на ленту
    show();
}

LedWS2812BforSTM32 Led;
