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
inertial BrainInertial = inertial();

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

  calibrateDrivetrain();

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

//base motor
motor leftFront = motor(PORT5, false);
motor leftBack = motor(PORT6, false);
motor rightFront = motor(PORT2, true);
motor rightBack = motor(PORT1, true);
motor_group leftDriveMotor = motor_group(leftFront, leftBack);
motor_group rightDriveMotor = motor_group(rightFront, rightBack);

// motor leftDriveMotor = motor(PORT1, true);
// motor rightDriveMotor = motor(PORT7, false);

drivetrain Drivetrain = drivetrain(leftDriveMotor, rightDriveMotor);

// roller motor
motor roller = motor(PORT3, true);


// Allows for easier use of the VEX Library
using namespace vex;

void PDrive(float Kp, float target, float distance, float speed) {
  float distanceMoved = 0;

  while (distanceMoved < distance)
  {
    leftDriveMotor.setPosition(0, turns);
    rightDriveMotor.setPosition(0, turns);

    float error = target - BrainInertial.rotation(degrees);
    float P = Kp * error;

    leftDriveMotor.setVelocity(speed + P, percent);
    rightDriveMotor.setVelocity(speed - P, percent);
    leftDriveMotor.spin(forward);
    rightDriveMotor.spin(forward);

    wait(20, msec);

    distanceMoved += (leftDriveMotor.position(turns) * 200 + rightDriveMotor.position(turns) * 200) / 2;
  }

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}

void PRotate(float Kp, float target, float speed) {
  while (BrainInertial.rotation(degrees) != target-1)
  {
    // leftDriveMotor.setPosition(0, turns);
    // rightDriveMotor.setPosition(0, turns);

    if (target < 180) {
      float error = BrainInertial.rotation(degrees) - (target - 360);
      float P = Kp * error;

      leftDriveMotor.setVelocity(-P, percent);
      rightDriveMotor.setVelocity(P, percent);
      leftDriveMotor.spin(forward);
      rightDriveMotor.spin(forward);
    }
    if (target < 180) {
      float error = target - BrainInertial.rotation(degrees);
      float P = Kp * error;

      leftDriveMotor.setVelocity(P, percent);
      rightDriveMotor.setVelocity(-P, percent);
      leftDriveMotor.spin(forward);
      rightDriveMotor.spin(forward);
    }

    wait(20, msec);
  }

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}


const int MAX_SPEED = 100;
const int MIN_SPEED = 10;
const float Kp_DRIVE = 2;
const float Kp_ROTATE = 2;

void Path() {

  // roller.setMaxTorque(100, percent);
  // roller.setVelocity(100, percent);
  // roller.spin(forward);


  // PDrive(Kp_DRIVE, 0, 2500, MAX_SPEED);
  PRotate(Kp_ROTATE, 270, MAX_SPEED);
  PDrive(Kp_DRIVE, -90, 2600, MAX_SPEED);

  // PRotate(Kp_ROTATE, 0, MAX_SPEED);
  // PDrive(Kp_DRIVE, 0, 200, MAX_SPEED);

  // PRotate(Kp_ROTATE, 180, MAX_SPEED);
  // PDrive(Kp_DRIVE, 180, 500, MAX_SPEED);

  // roller.setMaxTorque(100, percent);
  // roller.setVelocity(-100, percent);
  // roller.spin(forward);

  // PDrive(Kp_DRIVE, 180, 1000, MAX_SPEED);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Path();
}