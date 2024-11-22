#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors ls;
Zumo32U4Motors motors;
unsigned int lsValues[3];
#define lsthreshold 1000
#define speed 100
int state = 0;
// Hej Søren!!!!!!!!!!

void turnRight();
void turnLeft();

void setup()
{
  ls.initThreeSensors();
}

void loop()
{
  ls.read(lsValues); // Read linesensor values

  switch (state) // Switch the operating state
  {
  case 0: // Case 0: Robot searching for matrix
    if (lsValues[0] < lsthreshold &&
        lsValues[1] < lsthreshold &&
        lsValues[2] < lsthreshold) // All linesensors are not detecting a line
    {
      motors.setSpeeds(speed, speed); // Drive forward
    }
    else if (lsValues[0] > lsthreshold) // Matrix is to the left or straight in front of the robot
    {
      state = 1;
    }
    else // Matrix is to the right of the robot
    {
      state = 2;
    }
    motors.setSpeeds(0, 0); // Stop the movement
    break;

  case 1:                          // Case 1: Follow line to the left of the robot
    if (lsValues[0] > lsthreshold) // Robot on line
    {
      turnRight(); // Turn off line
    }
    else
    {
      motors.setSpeeds(speed, speed + 30); // Robot going forward and slightly left
    }
    break;

  case 2:                          // Case 1: Follow line to the left of the robot
    if (lsValues[2] > lsthreshold) // Robot on line
    {
      turnLeft(); // Turn off line
    }
    else
    {
      motors.setSpeeds(speed + 30, speed); // Robot going forward and slightly left
    }
    break;

  case 3:

  default:
    break;
  }
}

void turnRight()
{
  motors.setSpeeds(0, -speed);
}

void turnLeft()
{
  motors.setSpeeds(-speed, 0);
}