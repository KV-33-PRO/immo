#include <WS2812B.h>

#define NUM_LEDS    6
//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

WS2812B strip = WS2812B(NUM_LEDS);

void setup() 
{
  strip.begin();     // Sets up the SPI
  strip.show();      // Clears the strip, as by default the strip data is set to all LED's off.
  //strip.setBrightness(8);
}

void loop() 
{
  //Головной свет и задний ход
  stripOnColor(strip.Color(255, 255, 255)); //White
  delay(2000);
  
  //Поворотник
  stripOnColorRun(strip.Color(255, 128, 0), 150); //Yelloy
  
  //Стоп мигающий
  stripOnColorBlink(strip.Color(255, 0, 0), 120); //Red blink
  
  //Стоп постоянный
  stripOnColor(strip.Color(255, 0, 0)); //Red
  delay(2000);

  //Выключить всё
  strip.clear();
  delay(500);
}


void stripOnColor(uint32_t c) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
      strip.setPixelColor(i, c);
  }
      strip.show();
}

void stripOnColorRun(uint32_t c, uint8_t wait) 
{
  strip.clear();
  for(int f=0; f<5;f++){
    for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
  strip.clear();
  }
}

void stripOnColorBlink(uint32_t c, uint8_t wait)  {
  for(int b = 0; b<5; b++){
      stripOnColor(c);
      delay(wait);
      strip.clear();
      delay(wait);
  }
  stripOnColor(c);
}
