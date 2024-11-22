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
  motors.setSpeeds(0, -speed);
}

void turnLeft()
{
  motors.setSpeeds(-speed, 0);
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
    else if (lsValues[0] > lsthreshold)
    {
      state = 1;
    }
    else
    {
      state = 2;
    }
    motors.setSpeeds(0, 0);
    break;

  case 1:
    if (lsValues[0] > lsthreshold)
    {
      turnRight();
    }
    else
    {
      motors.setSpeeds(speed, speed + 30);
    }
    break;

  case 2:
    if (lsValues[2] > lsthreshold)
    {
      turnLeft();
    }
    else
    {
      motors.setSpeeds(speed + 30, speed);
    }
    break;
  
  case 3:
    
  default:
    break;
  }
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}