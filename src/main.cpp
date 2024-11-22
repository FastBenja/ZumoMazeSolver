#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors ls;
Zumo32U4Motors motors;

//Hej Søren!!!!!!!!!!

// put function declarations here:
int m(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}