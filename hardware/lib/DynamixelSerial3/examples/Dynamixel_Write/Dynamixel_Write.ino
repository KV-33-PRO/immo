#include <DynamixelSerial3.h>

void setup(){
Dynamixel.begin(1000000,PB12);  // Inicialize the servo at 1Mbps and Pin Control 2
delay(1000);
}

void loop(){
  Dynamixel.ledStatus(14,ON);
  delay(1000);
  Dynamixel.ledStatus(14,OFF);
delay(1000);

}
