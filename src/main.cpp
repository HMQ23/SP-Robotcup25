#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// Include the IQ Library
#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  vexcode_initial_drivetrain_calibration_completed = true;
  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

void vexcodeInit() {

  // Calibrate the Drivetrain
  calibrateDrivetrain();

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration


// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();

motor ratchetMotor7 = motor(PORT7, false);
motor ratchetMotor8 = motor(PORT8, true);
motor_group ratchetMotorGroup = motor_group(ratchetMotor7, ratchetMotor8);

motor leftDriveMotor = motor(PORT1, false);
motor rightDriveMotor = motor(PORT2, true);

// const float Kp = 20;
// const int MAX_SPEED = 100;
// const int MIN_SPEED = 0;



// Allows for easier use of the VEX Library
using namespace vex;


// float PDrive(float Kp, float targetSpeed) {
//   float PGain = Kp * targetSpeed;
  
//   if PGAin >
//   float output = max(min(PGain, MAX_SPEED), -MAX_SPEED);
  
//   return output;
// }


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  ratchetMotorGroup.setMaxTorque(100.0, percent);
  // ratchetMotorGroup.setVelocity(100.0, percent);

  
  float leftPos;
  float rightPos;
  leftPos = Controller.AxisA.position() + Controller.AxisC.position();
  rightPos = Controller.AxisA.position() - Controller.AxisC.position();

  leftDriveMotor.setVelocity(leftPos, percent);
  rightDriveMotor.setVelocity(rightPos, percent);

  leftDriveMotor.spin(forward);
  rightDriveMotor.spin(forward);


  if (Controller.ButtonRUp.pressing()) {
    ratchetMotorGroup.setVelocity(100.0, percent);
  }
  else {
    ratchetMotorGroup.setVelocity(0.0, percent);
  }

  if (Controller.ButtonRDown.pressing()) {
    ratchetMotorGroup.setVelocity(-100.0, percent);
  }
  else {
    ratchetMotorGroup.setVelocity(0.0, percent);
  }

  wait(20, msec);
}