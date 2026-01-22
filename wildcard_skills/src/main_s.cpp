/* //TODO: my bio hw
TODO: Create an autonomous program (pull start from main_c)
//TODO: Disable auton after a minute
//TODO: Create a switch ui for the driver (or me or whoever's doing skills)
*/
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <string>
#include "vex.h"

using namespace vex;
using std::string;
using std::cout;
using std::endl;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.
// PORT 4 IS RESERVED ----- FORTNITE

// Independent motors
motor scoreMotor = motor(PORT10, ratio6_1, true);
motor intakeMotor = motor(PORT3, ratio6_1, true);

// Drivetrain motors
motor leftMotorA = motor(PORT11, ratio6_1, false);
motor leftMotorB = motor(PORT1, ratio6_1, false);
motor leftMotorC = motor(PORT19, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT7, ratio6_1, true);
motor rightMotorB = motor(PORT20, ratio6_1, true);
motor rightMotorC = motor(PORT8, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 11.43, 12.953999999999999, mm, 1.3333333333333333);

controller Controller1 = controller(primary);

// generating and setting random seed
void initializeRandomSeed()
{
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}




void vexcodeInit()
{

  // Initializing random seed.
  initializeRandomSeed();
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName)
{
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

// Define a continuous timer to check 
// dig in you twin - jack
timer t;

// A flag checking for whether a function has run for a minute
bool minute(){
  return t.time(sec)<=20;
}

//Macro for running code after every line
#define TC(x) do {x; if (minute()) return;} while(false);

// Initialise the controller task --> Controller1
task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration

/* #region project info */
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main(S).cpp                                               */
/*    Author:       TV                                                        */
/*    Created:      1/21/2026, 1:05:06 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* #endregion */

//* Constants and Globals
const float bufferS = 2.25;                   // Buffer time in seconds
const int buffer2 = bufferS * 1000 + 10;     // Buffer time in milliseconds + 10 extra for safety
const int intakeSpeed = 15;                 // Intake motor speed
const int scoreSpeed = 80;                 // Scoring motor speed
const int counterSpeed = 20;              // Counter motor speed
const int turnSpeed = 50;                // Drivetrain turning speed
const int tile = 600;                   // One tile distance in mm 
const int driveSpeed = 100;            // Drivetrain speed
bool autonFlag = false;               // Check for autonomous or driving control
bool divineGeneralMahoraga = false;  //! OBLITERATE (literlly decimates the code on summon)


//* Misc functions to call later 
void buffer() {
  //! The legendary fake init function that does nothing
  //* but gives time to setup -- robot experiences inconsistent communication issues when removed
  timer t;
  t.reset();
  int i = 0;

  while (t.time(sec) <= bufferS) {
    //Simple loading animation just for filler
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Parthogenesis.init()");    
    string dots = string(i, '.');
    Brain.Screen.print(dots.c_str());
    i = (i + 1) % 4;
    wait(250, msec);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,21);
  }
  //Remove loading text after buffer completed
  screenReset();
}

void driveSetup() {
  // Motor setup
  Drivetrain.setDriveVelocity(driveSpeed, percent);
  Drivetrain.setTurnVelocity(turnSpeed, percent);
  intakeMotor.setVelocity(intakeSpeed, percent);
  scoreMotor.setVelocity(scoreSpeed, percent);
}

void screenReset() {
  // Resets the brain screen to default state
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}

void stopAllMotors() {
  // Stops all motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  intakeMotor.stop();
  scoreMotor.stop();
  Drivetrain.stop();
}

void go(double distance) {
  // Simplifies below function to simply go(distance);
  Drivetrain.driveFor(forward, distance,mm);
}

void goTile(int tilecount) {
  //an alternative go(distance) that deals in tiles rather mm
  Drivetrain.driveFor(forward,tilecount*tile,mm);
}


//* UI on brain to allow autonomous or driving selection
void ui() {
  screenReset();
  Brain.Screen.print("Press LEFT for driving, RIGHT for autonomous");
  Brain.Screen.newLine();
  while (true){
   if (Controller1.ButtonLeft.pressing() == true)
    {
      Brain.Screen.print("LEFT pressed; driving init...");
      autonFlag = false;
      break;
    } else if (Controller1.ButtonDown.pressing() == true){
      Brain.Screen.print("RIGHT pressed; auton init...");
      autonFlag = true;
      break;
    }
  }
  return;
}

//* Drive and Autonomous functions
void drive() {
  // Driver control code
  screenReset();
  
  Brain.Screen.print("Driver Control Initialized");
  while (true) {
    // Checks if  
    if (!minute()) break;

    // Scoring control
    if (Controller1.ButtonR1.pressing()) {
      intakeMotor.setVelocity(intakeSpeed, percent);
      scoreMotor.setVelocity(counterSpeed, percent);
      intakeMotor.spin(forward);
      scoreMotor.spin(reverse); // to prevent jamming while intaking

     }
    

    // Intake control
    else if (Controller1.ButtonL1.pressing()) {
      scoreMotor.setVelocity(scoreSpeed, percent);
      scoreMotor.spin(forward);


    } 
    // Stops motors in absence of input
    else {
      scoreMotor.stop();
      intakeMotor.stop();
    }

    //! Add onto ALL while loops to prevent wasted CPU cycles
    wait(20, msec);
  }
  /*
  * With this treasure...
  * I summon...
  * Divine General
  ! Mahoraga
  */
  divineGeneralMahoraga = true;
  return;
}



void autonomous(){
  t.reset();
  TC(wait(50,msec));
  TC(RemoteControlCodeEnabled = false);
  TC(screenReset());
  TC(Brain.Screen.print("Autonomous Initialized"));
  TC(goTile(1));

}

int main() {
  //Initialize rand() DO NOT REMOVE
  vexcodeInit();
  
  //Begin Project Code
  
  ui();                                //Ask user which program to run
  t.reset();
  //Decide on either auton or driver control based on controller input
  while (true){
    if (divineGeneralMahoraga == true) {
    // Once the timer is up //! summon mahoraga
      return 0;
    } 
    else if (autonFlag == false /*on left press*/){
      drive();
    } 
    else if (autonFlag == true /*on right press*/) {
      autonomous();  
    }
   

  }
  
}
