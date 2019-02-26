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

//INDICATION STATUS
#define POSITION_LAMPS     0
#define DIPPED_BEAM_LAMPS  1
#define HIGH_BEAM_LAMPS    2
#define BRAKE_LAMPS        3
#define BACK_LAMPS         4
#define TURN_LEFT_LAMPS    5
#define TURN_RIGHT_LAMPS   6
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
        setSideColorLeds(FRONT, 0, COUNT_LEDS_HEADLIGHT-1, _white_low);
        setSideColorLeds(REAR, 0, COUNT_LEDS_HEADLIGHT-1, _red_low);
    }
    else
    {
        setSideColorLeds(ALL, 0, COUNT_LEDS_HEADLIGHT-1, _black);
    }
}

void LedWS2812BforSTM32::head(){
    if(_indication_status[DIPPED_BEAM_LAMPS])
    {
        setSideColorLeds(FRONT, 0, COUNT_LEDS_HEADLIGHT-1, _white_middle);
    }
    if(_indication_status[HIGH_BEAM_LAMPS])
    {
        setSideColorLeds(FRONT, 0, COUNT_LEDS_HEADLIGHT-1, _white_high);
    }
}

void LedWS2812BforSTM32::backAndBrake(){
    if(_indication_status[BACK_LAMPS] && _indication_status[BRAKE_LAMPS])
    {
        setSideColorLeds(REAR, 0, LED_SEPARATING_2_MODE-1, _white_high);
        setSideColorLeds(REAR, LED_SEPARATING_2_MODE, COUNT_LEDS_HEADLIGHT-1, _red_high);   //Просто включается (не моргает)
    }
    else
    {
        if(_indication_status[BACK_LAMPS])
        {
        setSideColorLeds(REAR, 0, COUNT_LEDS_HEADLIGHT-1, _white_high);
        }
        if(_indication_status[BRAKE_LAMPS])
        {
        setSideColorLeds(REAR, 0, COUNT_LEDS_HEADLIGHT-1, _red_high);  //Просто включается (не моргает)
        }
    }
}

void LedWS2812BforSTM32::turn(){
    int side = 5;
    if(_indication_status[TURN_LEFT_LAMPS] || _indication_status[TURN_RIGHT_LAMPS])
    {
        _turn_status=true;
    }

    if(_indication_status[TURN_LEFT_LAMPS] && _indication_status[TURN_RIGHT_LAMPS])
    {
        side=ALL;
    }
    else
    {
        if(_indication_status[TURN_LEFT_LAMPS])
        {
            side=LEFT;
        }
        if(_indication_status[TURN_RIGHT_LAMPS])
        {
            side=RIGHT;
        }
    }
    setSideColorLeds(side, LED_START_FOR_TURN, COUNT_LEDS_HEADLIGHT-1, _black);
    if(_turn_count<COUNT_LEDS_HEADLIGHT && _turn_status == true)
    {
        setSideColorLeds(side, LED_START_FOR_TURN, _turn_count, _yelloy);
    }
    else
    {
        _turn_count=LED_START_FOR_TURN;
        _turn_status=false;
    }
    if(millis() - _turn_last_time >= RATE_TURN_MS)
    {
        _turn_count++;
        _turn_last_time = millis();
    }
}

void LedWS2812BforSTM32::flaser(){
    if(_indication_status[FLASHER_LAMPS])
    {
        setSideColorLeds(ALL, 0, COUNT_LEDS_HEADLIGHT-1, _black);
        if(millis() - _flasher_last_time >= RATE_FLASHER_MS)
        {
            if(_flasher_count==3)
            {
                _flasher_side==LEFT ? _flasher_side=RIGHT : _flasher_side=LEFT;
                _flasher_count=0;
            }

            if(_flasher_status==false)
            {
                setSideColorLeds(_flasher_side, 0, COUNT_LEDS_HEADLIGHT-1, (_flasher_side == LEFT ? _blue : _red_high));
            }
            else
            {
                setSideColorLeds(ALL, 0, COUNT_LEDS_HEADLIGHT-1, _black);
                _flasher_count++;
            }
            _flasher_status=!_flasher_status;
            _flasher_last_time = millis();
        }
    }
    else
    {
        _flasher_side=LEFT;
        _flasher_count=0;
    }
}

void LedWS2812BforSTM32::show(){
    for(int lamps = 0; lamps<NUM_LEDS/COUNT_LEDS_HEADLIGHT; lamps++)
    {
        for(int led = 0; led<COUNT_LEDS_HEADLIGHT; led++){
            _strip.setPixelColor(_leds[lamps][led], _leds_color[lamps][led]);
        }
    }
    _strip.show();
}

// Public Methods //////////////////////////////////////////////////////////////

LedWS2812BforSTM32::LedWS2812BforSTM32()
{
    _yelloy = _strip.Color(255, 128, 0);
    _blue = _strip.Color(0, 0, 255);
    _red_high = _strip.Color(255, 0, 0);
    _red_low = _strip.Color(70, 0, 0);
    _white_high = _strip.Color(255, 255, 255);
    _white_middle = _strip.Color(150, 150, 180);
    _white_low = _strip.Color(70, 70, 70);
    _black = _strip.Color(0, 0, 0);

    _turn_count = 0;
    _turn_status=TURN_OFF;

    for(int status = 0; status<COUNT_INDICATION_STATUS; status++){
        _indication_status[status] = false;
    }
    setSideColorLeds(ALL, 0, COUNT_LEDS_HEADLIGHT-1, _black);
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

String LedWS2812BforSTM32::getData()
{
    String str = "";
    int count = 0;
    while(count<NUM_LEDS)
    {
        for(int lamps = 0; lamps<NUM_LEDS/COUNT_LEDS_HEADLIGHT; lamps++)
        {
            for(int led = 0; led<COUNT_LEDS_HEADLIGHT; led++){
                if(count==_leds[lamps][led]){
                    if(_leds_color[lamps][led]==_white_low)
                        str += "б";
                    if(_leds_color[lamps][led]==_red_low)
                        str += "к";
                    if(_leds_color[lamps][led]==_white_middle)
                        str += "W";
                    if(_leds_color[lamps][led]==_white_high)
                        str += "Б";
                    if(_leds_color[lamps][led]==_red_high)
                        str += "К";
                    if(_leds_color[lamps][led]==_black)
                        str += "Ч";
                    if(_leds_color[lamps][led]==_yelloy)
                        str += "Ж";
                    if(_leds_color[lamps][led]==_blue)
                        str += "С";
                    count++;
                    if(count == NUM_LEDS/2 || count == NUM_LEDS)
                    {
                        str += "\r\n";
                    }

                    if(count == NUM_LEDS/4 || count == (NUM_LEDS/2+NUM_LEDS/4))
                    {
                        str += " ";
                    }
                }
            }
        }
    }
    return str;
}

void LedWS2812BforSTM32::init()
{
    _strip.begin();     // Sets up the SPI
    _strip.show();      // Clears the strip, as by default the strip data is set to all LED's off.
    //_strip.setBrightness(0);
}

LedWS2812BforSTM32 Led;
