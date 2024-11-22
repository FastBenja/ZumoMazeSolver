#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors ls;
Zumo32U4Motors motors;
unsigned int lsValues[3];
#define lsthreshold 1000
#define speed 100
int state = 0;
// Hej Søren!!!!!!!!!!

// put function declarations here:
int m(int, int);

void setup()
{
  // put your setup code here, to run once:
  ls.initThreeSensors();
}

void turnRight()
{
  motors.setSpeeds(speed, -speed);
}

void turnLeft()
{
  motors.setSpeeds(-speed, speed);
}

void loop()
{
  // put your main code here, to run repeatedly:
  ls.read(lsValues);

  switch (state)
  {
  case 0:
    if (lsValues[0] < lsthreshold && lsValues[1] < lsthreshold && lsValues[2] < lsthreshold)
    {
      motors.setSpeeds(speed, speed);
    }
    else
    {
      motors.setSpeeds(0, 0);
      state = 1;
    }
    break;

  case 1:
    if (lsValues[0] > lsthreshold)
    {
      turnRight();
    }
    else
    {
      forward slightly left
    }
    break;

  default:
    break;
  }

  if (lsValues[0] > lsthreshold)
  {
    follow line on left side;
  }
  else if (lsValues[2] > lsthreshold)
  {
    follow line on right side;
  }
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}