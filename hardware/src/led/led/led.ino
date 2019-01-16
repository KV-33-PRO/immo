#include <WS2812B.h>

#define NUM_LEDS    20
#define COUNT_LEDS_HEADLIGHT    5
//#define PIN_LEDS    PA7     //Library uses SPI1. Connect the WS2812B data input to MOSI on your board. (STM32: PA7)

WS2812B strip = WS2812B(NUM_LEDS);

uint32_t yelloy = strip.Color(255, 128, 0);
uint32_t red = strip.Color(255, 0, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t white = strip.Color(255, 255, 255);
uint32_t black = strip.Color(0, 0, 0);

int leds[4][COUNT_LEDS_HEADLIGHT] = {{4, 3, 2, 1, 0}, {5, 6, 7, 8, 9}, {14 ,13 ,12, 11, 10}, {15, 16, 17, 18, 19}};  //0 - левая передная, 1 - правая передняя, 2 - левая задняя, 3 - правая задняя

void setup()
{
  strip.begin();     // Sets up the SPI
  strip.show();      // Clears the strip, as by default the strip data is set to all LED's off.
  //strip.setBrightness(8);
}

void loop()
{
  headLamps(0);
  delay(2000);

  blinkerLamps(3, 1, true);

  stopBlink();
  delay(2000);

  stripShowColor(4, 0, 1, 2, 3, black);
  delay(500);

  blinkerLamps(3, 0, false);

  blinkerLamps(3, 1, false);

  stripShowColor(4, 0, 1, 2, 3, black);
  delay(2000);

  viutrtr();
  delay(1000);

  stripShowColor(4, 0, 1, 2, 3, black);
  delay(1000);
}

//Мигание поворотником
void blinkerLamps(int count_blink, int side, bool fail_safes)
{
  int wait = 150;
  int lamp1;
  int lamp2;
  int lamp3;
  int lamp4;
  int count_lamps = 2;

  if (fail_safes) {
    lamp1 = 0;
    lamp2 = 1;
    lamp3 = 2;
    lamp4 = 3;
    count_lamps = 4;
  }
  else
  {
    if (side == 0) {
      lamp1 = 0;
      lamp2 = 2;
    }
    if (side == 1) {
      lamp1 = 1;
      lamp2 = 3;
    }
  }
  for (int count = 0; count < count_blink; count++) {
    blinker(count_lamps, lamp1, lamp2, lamp3, lamp4);
  }
}

//Мигание последовательно (поворотники)
void blinker(int count_lamps, int lamp1, int lamp2, int lamp3, int lamp4)
{
  int wait = 150;
  if (count_lamps == 2) {
    stripShowColor(count_lamps, lamp1, lamp2, 0, 0, black);
  }
  if (count_lamps == 4) {
    stripShowColor(count_lamps, lamp1, lamp2, lamp3, lamp4, black);
  }
  delay(wait);
  for (int i = 0; i < COUNT_LEDS_HEADLIGHT; i++)
  {
    if (count_lamps == 2) {
      strip.setPixelColor(leds[lamp1][i], yelloy);  //Передний фонарь
      strip.setPixelColor(leds[lamp2][i], yelloy);  //Задний фонарь
    }
    if (count_lamps == 4) {
      strip.setPixelColor(leds[lamp1][i], yelloy);  //Передний фонарь
      strip.setPixelColor(leds[lamp2][i], yelloy);  //Задний фонарь
      strip.setPixelColor(leds[lamp3][i], yelloy);  //Передний фонарь
      strip.setPixelColor(leds[lamp4][i], yelloy);  //Задний фонарь
    }
    strip.show();
    delay(wait);
  }
  if (count_lamps == 2) {
    stripShowColor(count_lamps, lamp1, lamp2, 0, 0, black);
  }
  if (count_lamps == 4) {
    stripShowColor(count_lamps, lamp1, lamp2, lamp3, lamp4, black);
  }
}

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

//Мигающий стоп
void stopBlink()
{
  int wait = 80;
  for (int b = 0; b <= 2; b++) {
    stripShowColor(2, 2, 3, 0, 0, red); //Задние фонари
    delay(wait);
    stripShowColor(2, 2, 3, 0, 0, black); //Задние фонари
    delay(wait);
  }
  stripShowColor(2, 2, 3, 0, 0, red); //Задние фонари
}

//Белый свет
void headLamps(int direction)
{
  if (direction == 0)
  {
    stripShowColor(2, 0, 1, 0, 0, white); //Передние фары
  }
  if (direction == 1)
  {
    stripShowColor(2, 2, 3, 0, 0, white); //Задние фонари
  }
}

//Виу-виу-тр-тр...
void viutrtr()
{
  int wait = 50;
  for (int b = 0; b <= 5; b++) {
    for (int b = 0; b <= 3; b++) {
      stripShowColor(2, 1, 3, 0, 0, red); //Задние фонари
      delay(wait);  
      stripShowColor(4, 0, 1, 2, 3, black); //Задние фонари
      delay(wait); 
    }
    for (int b = 0; b <= 3; b++) {
      stripShowColor(2, 0, 2, 0, 0, blue); //Задние фонари
      delay(wait);
      stripShowColor(4, 0, 1, 2, 3, black); //Задние фонари
      delay(wait); 
    }
  }
  stripShowColor(4, 0, 1, 2, 3, black); //Задние фонари
}
