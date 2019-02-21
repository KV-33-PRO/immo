#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "Led.h"

#define FL 0
#define FR 1
#define RL 2
#define RR 3

#define FRONT 0
#define REAR  1


// Private Methods //////////////////////////////////////////////////////////////

// Включить фонари на нужный цвет
void stripShowColor(int count_lamps, int lamp1, int lamp2, int lamp3, int lamp4, uint32_t color)
{
  for (int i = 0; i < COUNT_LEDS_HEADLIGHT; i++)
  {
    if (count_lamps == 2) {
      strip.setPixelColor(leds[lamp1][i], color);  //Передний фонарь
      strip.setPixelColor(leds[lamp2][i], color);  //Задний фонарь
    }
    if (count_lamps == 4) {
      strip.setPixelColor(leds[lamp1][i], color);  //Передний фонарь
      strip.setPixelColor(leds[lamp2][i], color);  //Задний фонарь
      strip.setPixelColor(leds[lamp3][i], color);  //Передний фонарь
      strip.setPixelColor(leds[lamp4][i], color);  //Задний фонарь
    }
  }
  strip.show();
}

void Led::setColorLeds(int side, int id_led, uint32_t color){
    if(side == FRONT)
    {
        strip.setPixelColor(_leds[FL][id_led], color);
        strip.setPixelColor(_leds[FR][id_led], color);
    }
    if(side == REAR)
    {
        strip.setPixelColor(_leds[RL][id_led], color);
        strip.setPixelColor(_leds[RR][id_led], color);
    }
}

// Public Methods //////////////////////////////////////////////////////////////

Led::Led()
{
    _strip = WS2812B(NUM_LEDS);
    _strip.begin();     // Sets up the SPI
    _strip.show();      // Clears the strip, as by default the strip data is set to all LED's off.
    //strip.setBrightness(8);

    _yelloy = strip.Color(255, 128, 0);
    _blue = strip.Color(0, 0, 255);
    _red_high = strip.Color(255, 0, 0);
    _red_low = strip.Color(100, 0, 0);
    _white_high = strip.Color(255, 255, 255);
    _white_middle = strip.Color(180, 180, 180);
    _white_low = strip.Color(100, 100, 100);
    _black = strip.Color(0, 0, 0);

    _leds[4][COUNT_LEDS_HEADLIGHT] = {{4, 3, 2, 1, 0}, {5, 6, 7, 8, 9}, {14 ,13 ,12, 11, 10}, {15, 16, 17, 18, 19}};  //0 - левая передная, 1 - правая передняя, 2 - левая задняя, 3 - правая задняя
}

void Led::head(uint8_t mode){
    for (int i=0; i<3; i++){
        if(mode == HEAD_HIGH_BEAM)
        {
            setColorLeds(FRONT, i, _white_high);
        }
        if(mode == HEAD_DIPPED_BEAM)
        {
            setColorLeds(FRONT, i, _white_middle);
        }
        if(mode == HEAD_OFF)
        {
            setColorLeds(FRONT, i, _black);
        }
    }
}

void Led::turn(uint8_t mode){

}

void Led::back(uint8_t mode){
    if(mode == BACK_ON)
    {
        setColorLeds(REAR, 3, _white_high);
    }
    if(mode == BACK_OFF)
    {
        setColorLeds(REAR, 3, _black);
    }
}

void Led::brake(uint8_t mode){
    for (int i=0; i<2; i++){
        if(mode == BRAKE_ON)
        {
            setColorLeds(REAR, i, _red_high);
        }
        if(mode == BRAKE_OFF)
        {
            setColorLeds(REAR, i, _black);
        }
    }
}

void Led::flaser(uint8_t mode){
    if(mode == FLASHER_ON)
    {
    int wait = 50;
    for (int b = 0; b <= 5; b++) {
      for (int b = 0; b <= 3; b++) {
        stripShowColor(2, 1, 3, 0, 0, red);
        delay(wait);
        stripShowColor(4, 0, 1, 2, 3, black);
        delay(wait);
      }
      for (int b = 0; b <= 3; b++) {
        stripShowColor(2, 0, 2, 0, 0, blue);
        delay(wait);
        stripShowColor(4, 0, 1, 2, 3, black);
        delay(wait);
      }
    }
    stripShowColor(4, 0, 1, 2, 3, black);
    }
    else
    {
        stripShowColor(4, 0, 1, 2, 3, black);
    }
}

void Led::position(uint8_t mode){
    for (int i=0; i<2; i++){
    if (mode == POSITION_LAMPS_ON){
        setColorLeds(FRONT, i, _white_low);
        setColorLeds(REAR, i, _red_low);
    }
    if (mode == POSITION_LAMPS_OFF){
    {
        setColorLeds(FRONT, i, _black);
        setColorLeds(REAR, i, _black);
    }
    }
}
