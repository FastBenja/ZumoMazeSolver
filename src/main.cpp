#include <Arduino.h>
#include <Zumo32U4.h>
#include <PID_v1.h>

// Define Variables we'll be connecting to
double Setpoint = 500, Input, Output;
// Specify the links and initial tuning parameters
// Kp=0.14, Ki=0.035, Kd=0.009 works well for outlimit -200, 200
// Kp = 0.14 * 10, Ki = 0.035 * 10, Kd = 0.009 * 10; with scaling and outlimit -1000 to 1000
double Kp = 0.14, Ki = 0.035, Kd = 0.009;

Zumo32U4LineSensors ls;
Zumo32U4Motors motors;
Zumo32U4OLED oled;
Zumo32U4ButtonA btnA;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned int lsValues[3];
#define target 500 // Target value for detecting a line
#define speed 80
#define LEFT lsValues[0]
#define RIGHT lsValues[2]
int state = 0;
// Hej Søren!!!!!!!!!!

void turnRight();
void turnLeft();
void calibrateLS();
void printStrings(String, String);
void followLine(unsigned int);

void setup()
{
  ls.initThreeSensors();
  Serial.begin(9600);
  pid.SetSampleTime(50);
  pid.SetOutputLimits(-speed, speed);
  // turn the PID on
  pid.SetMode(AUTOMATIC);

  calibrateLS();
}

void loop()
{
  ls.readCalibrated(lsValues); // Read linesensor values

  // Serial.println("Ls1: " + String(lsValues[0]) + " Ls2: " + String(lsValues[1]) + " Ls3: " + String(lsValues[2]));
  // Serial.println(lsValues[0]);

  switch (state) // Switch the operating state
  {
  case 0:                                                                     // Case 0: Robot searching for matrix
    if (lsValues[0] < target && lsValues[1] < target && lsValues[2] < target) // All linesensors are not detecting a line
    {
      motors.setSpeeds(speed, speed); // Drive forward
      break;
    }
    else if (lsValues[0] > target) // Matrix is to the left or straight in front of the robot
    {
      state = 1;
    }
    else // Matrix is to the right of the robot
    {
      state = 2;
    }
    motors.setSpeeds(0, 0); // Stop the movement
    break;

  case 1: // Case 1: Follow line to the left of the robot
    followLine(LEFT);
    break;

  case 2: // Case 1: Follow line to the left of the robot
    followLine(RIGHT);
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

void calibrateLS()
{
  printStrings(F("Cali-"), F("brate LS"));

  for (uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }
    ls.calibrate();
  }
  motors.setSpeeds(0, 0);

  printStrings(F("Done!"), F("Pres A"));
  btnA.waitForButton();
}

void printStrings(String str1, String str2)
{
  oled.clear();
  oled.print(str1);
  oled.gotoXY(0, 1);
  oled.print(str2);
}

void followLine(unsigned int sens)
{
  Input = sens;
  ls.readCalibrated(lsValues);
  pid.Compute(); // Compute pid output.
  int speedLeft = speed - Output;
  int speedRight = speed + Output;

  //int speedLeft = speed - speed * (Output / 1000);
  //int speedRight = speed + speed * (Output / 1000);
  //motors.setSpeeds(speedLeft, speedRight);
  if (sens >800 && lsValues[1] > 800)
  {
  motors.setSpeeds(speed, -speed);
  delay(2000);
  }
  motors.setSpeeds(speedLeft, speedRight);

  Serial.println("Reading: " + String(sens) + "\tOut: " + String(Output) + "\tLeft: " + String(speedLeft) + " \tRight: " + String(speedRight));
}
