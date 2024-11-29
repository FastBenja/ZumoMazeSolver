/*

Instructions:
======================================

This code should be uploaded to the Zumo with a OLED screen.
The Zumo is in the OFF state.
The Zumo is placed on top of one of the lines.
The Zumo is turned ON.
The Zumo will start spinning and calibrating.
When the Zumo is stationarry, move it to the desired starting position outside the maze.
Press buton A to start the Zumo.
*/

#include <Arduino.h>
#include <Zumo32U4.h>
#include <PID_v1.h>

/*
Kp=0.14, Ki=0.035, Kd=0.009 works well for outlimit -200, 200
Kp = 0.14 * 10, Ki = 0.035 * 10, Kd = 0.009 * 10; with scaling and outlimit -1000 to 1000
*/
double Kp = 0.15, Ki = 0.027, Kd = 0.005; // PID parameters

#define speed 90          // Robot speed (if changed a lot, new PID values will have to be found, keep around 100)
#define LineThreshold 300 // Threshold for detecting a solid line
#define turnTime 1900     // The time that the robot should turn in an inside corner

int state = 0;                        // Variable to describe the state that the robot is in
int side;                             // Variable to store the side that that the robot is driving
unsigned int lsValues[3];             // Array to store readings from linesensors
#define LEFT 0                        // Define alias for left
#define RIGHT 2                       // Define alias for right
double Setpoint = 500, Input, Output; // Variabels for PID

Zumo32U4LineSensors ls;                                  // Linesensor object
Zumo32U4Motors motors;                                   // Motors object
Zumo32U4OLED oled;                                       // Screen object
Zumo32U4ButtonA btnA;                                    // Button object
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // PID object

// Function definitions
void turnRight();
void turnLeft();
void calibrateLS();
void printStrings(String, String);
void followLine(unsigned int);
void mazeTurn(int);
int faceLine();

void setup()
{
  Serial.begin(9600);    // Initialize serial for debugging
  ls.initThreeSensors(); // Initialize 3 linesensors

  pid.SetSampleTime(50);              // Set sampletime for PID
  pid.SetOutputLimits(-speed*2, speed*2); // Set limits for PID output
  pid.SetMode(AUTOMATIC);             // turn the PID on

  calibrateLS(); // Run the linesensor calibration program
}

/**
 * \brief Main loop
 */
void loop()
{
  ls.readCalibrated(lsValues); // Read linesensor values

  // Debugging
  //  Serial.println("Ls1: " + String(lsValues[0]) + " Ls2: " + String(lsValues[1]) + " Ls3: " + String(lsValues[2]));

  switch (state) // Switch the operating state
  {
  case 0: // Case 0: Robot searching for Maze
    side = faceLine();

    if (side == LEFT) // Maze is to the left or straight in front of the robot
    {
      motors.setSpeeds(speed, -speed); // Turn right
      delay(1100);                     // Turn 90 degrees
      state = 1;                       // Prepare to follow line on the left
    }
    if (side == RIGHT) // Maze is to the right of the robot
    {
      motors.setSpeeds(-speed, speed); // Turn left
      delay(1100);                     // Turn 90 degrees
      state = 2;                       // Prepare to follow line on the right
    }
    motors.setSpeeds(0, 0); // Stop the movement
    break;

  case 1: // Case 1: Wait for all linesensors to be off of the line
    while (lsValues[0] > LineThreshold)
    {
      ls.readCalibrated(lsValues); // Update linesensor values
      motors.setSpeeds(0, -speed); // Turn right
    }

    state = 3; // Start line following to the left of the robot
    break;

  case 2: // Case 2: Wait for all linesensors to be off of the line
    while (lsValues[2] > LineThreshold)
    {
      ls.readCalibrated(lsValues); // Update linesensor values
      motors.setSpeeds(-speed, 0); // Turn left
    }

    state = 4; // Start line following to the right of the robot
    break;

  case 3: // Case 3: Follow line to the left of the robot
    followLine(LEFT);
    break;

  case 4: // Case 4: Follow line to the right of the robot
    followLine(RIGHT);
    break;
  }
}

/**
 * \brief Starts to turn right with the globally defined speed
 */
void turnRight()
{
  motors.setSpeeds(speed, -speed);
}

/**
 * \brief Starts to turn left with the globally defined speed
 */
void turnLeft()
{
  motors.setSpeeds(-speed, speed);
}

/**
 * \brief Place the robot on a tape line and run this function.
 * When the program returns from the function, the linesensors have been calibrated.
 */
void calibrateLS()
{
  printStrings(F("Cali-"), F("brate LS")); // Print a calibration message

  for (uint16_t i = 0; i < 120; i++) // Run the calibration
  {
    if (i > 30 && i <= 90) // Rotate left
    {
      motors.setSpeeds(-200, 200);
    }
    else // Rotate right
    {
      motors.setSpeeds(200, -200);
    }
    ls.calibrate(); // Calibrate
  }
  motors.setSpeeds(0, 0); // Stop

  printStrings(F("Done!"), F("Pres A")); // Print that the calibration is done
  btnA.waitForButton();                  // Wait for user pressing button A
}

/**
 * \brief Prints text to the OLED
 * \param str1 String to print on the upper line of the OLED
 * \param str2 String to print on the lower line of the OLED
 */
void printStrings(String str1, String str2)
{
  oled.clear();
  oled.print(str1);
  oled.gotoXY(0, 1);
  oled.print(str2);
}

/**
 * \brief Main line following rutine
 * \param side Specify the side that the line is, compared to the orientation of the robot.
 */
void followLine(unsigned int side)
{
  Input = lsValues[side];      // Input for PID is the sensor which corresponds to the correct side
  ls.readCalibrated(lsValues); // Keep line sensor values updated
  pid.Compute();               // Compute pid output
  int speedLeft;               // Initialize a variable to hold the calculated output for the left motor
  int speedRight;              // Initialize a variable to hold the calculated output for the right motor

  if (side == RIGHT) // If robot is following a line to the right of itself
  {
    speedLeft = speed + Output;  // Left is more positive that right side for positive PID output
    speedRight = speed - Output; //
    mazeTurn(side);              // Check for inside turns
  }

  if (side == LEFT) // If robot is following a line to the right of itself
  {
    speedLeft = speed - Output;  // Left is more negative that right side for positive PID output
    speedRight = speed + Output; //
    mazeTurn(side);              // Check for inside turns
  }

  motors.setSpeeds(speedLeft, speedRight); // Set the motorspeeds that have been calculated

  // Debugging
  // Serial.println("Reading: " + String(lsValues[side]) + "\tOut: " + String(Output) + "\tLeft: " + String(speedLeft) + " \tRight: " + String(speedRight));
}

/**
 * \brief Should be called very often. If encountering an inside turn, the robot will execute a turn based on the specified direction
 * \param dir Specify the direction to turn
 */
void mazeTurn(int dir)
{
  if (lsValues[dir] > LineThreshold && lsValues[1] > LineThreshold) // Encountered a line head on
  {
    if (dir == RIGHT) // Going right, turn left
    {
      motors.setSpeeds(-80, 80);
    }
    if (dir == LEFT) // Going left, turn right
    {
      motors.setSpeeds(80, -80);
    }
    delay(turnTime); // Wait for the robot to turn
  }
}

/**
 * \brief Makes the robot go forward until it encounters a line.
 * When a line is found the robot will turn itself perpendicular to the line.
 * When the robot is perpendicular, the function returns.
 * \returns The side that the robot first encountered the line on */
int faceLine()
{
  bool sideDetermined = false;
  int side = LEFT; // Default side is left, this is declared here to make sure that if the robot is starting on a line it still has a valid side.
  bool leftHasTouched = false;
  bool rightHasTouched = false;

  while (!leftHasTouched || !rightHasTouched) // While both linesensors are not on a line
  {
    ls.readCalibrated(lsValues); // Keep line sensor values updated
    Serial.println("LS:0 " + String(lsValues[0]) + "\t LS2: " + String(lsValues[2]));
    motors.setSpeeds(speed, speed); // Drive forward

    if (!sideDetermined && lsValues[0] > LineThreshold) // First encountered line on the left
    {
      side = LEFT;
      sideDetermined = true;
    }
    if (!sideDetermined && lsValues[2] > LineThreshold) // First encountered line on the right
    {
      side = RIGHT;
      sideDetermined = true;
    }
    if (lsValues[1] > LineThreshold)
    {
      motors.setSpeeds(-speed, -speed);
      delay(700);
      motors.setSpeeds(-speed, speed);
      delay(500);
      while (lsValues[2] < LineThreshold)
      {
        ls.readCalibrated(lsValues); // Keep line sensor values updated
        motors.setSpeeds(speed + 20, speed);
      }
    }

    while (lsValues[0] > LineThreshold) // While left sensor is seeing a line
    {
      leftHasTouched = true;
      ls.readCalibrated(lsValues); // Keep line sensor values updated
      motors.setSpeeds(-speed, 0); // Reverse with left belt
    }
    while (lsValues[2] > LineThreshold) // While right sensor is seeing a line
    {
      rightHasTouched = true;
      ls.readCalibrated(lsValues); // Keep line sensor values updated
      motors.setSpeeds(0, -speed); // Reverse with right belt
    }
  }
  return side;
}
