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


// ========== HARDWARE CONFIGURATION ==========

// Base motors - Configured as motor groups for left and right sides
motor leftFront = motor(PORT5, false);    // Left front motor, not reversed
motor leftBack = motor(PORT6, false);     // Left back motor, not reversed
motor rightFront = motor(PORT2, true);    // Right front motor, reversed
motor rightBack = motor(PORT1, true);     // Right back motor, reversed

// Motor groups for easier control of left and right sides
// Drivetrain object for built-in drive functions
motor_group leftDriveMotor = motor_group(leftFront, leftBack);
motor_group rightDriveMotor = motor_group(rightFront, rightBack);
drivetrain Drivetrain = drivetrain(leftDriveMotor, rightDriveMotor);

// Roller motor for intake or other mechanisms
motor roller = motor(PORT3, true);        // Roller motor, reversed


// ========== CONSTANTS FOR TUNING ==========
const int MAX_SPEED = 100;    // Maximum motor speed percentage (0-100%)
const int MIN_SPEED = 10;     // Minimum motor speed to overcome friction
const float Kp_DRIVE = 10;     // Proportional gain for driving straight
const float Kp_ROTATE = 2;    // Proportional gain for rotating

const float MaxCount = 25;
float count = 0;


// Allows for easier use of the VEX Library
using namespace vex;

// ========== PROPORTIONAL DRIVE FUNCTION ==========

//Check if robot can still move
float Dist = 0;
void LaternalCheck() {
  float PrevDist = 0;
  while(true) {    
    wait(0.1, seconds);
    //Compare the difference between the last measured distance and this one, if it is too small, the robot is most likely stuck or have hit a wall
    if(abs(PrevDist - Dist) < 0.1) {
      count += 1;
    }
    else {
      count = 0;
    }
    PrevDist = Dist;
  }
}

/**
 * Drives the robot straight for a specified distance while maintaining heading
 * Kp - Proportional gain constant (higher = stronger corrections)
 * target - Desired heading angle in degrees to maintain
 * distance - Total distance to travel (units based on wheel circumference)
 * speed - Base driving speed percentage (0-100%)
 */
void PDrive(float Kp, float target, float distance, float speed) {
  float distanceMoved = 0; // Track how far we've traveled

  // Continue driving until we reach the target distance
  while (distanceMoved < distance) {
    leftDriveMotor.setPosition(0, turns);
    rightDriveMotor.setPosition(0, turns);

    // Calculate heading error: difference between target and current heading
    float error = target - BrainInertial.rotation(degrees);
    // Calculate proportional correction value
    float P = Kp * error;

    // Apply correction to motor speeds:
    // - Left motor: base speed + correction
    // - Right motor: base speed - correction
    // This causes the robot to turn toward the target heading
    leftDriveMotor.setVelocity(speed + P, percent);
    rightDriveMotor.setVelocity(speed - P, percent);
    leftDriveMotor.spin(forward);
    rightDriveMotor.spin(forward);

     wait(0.1, seconds);

    // Calculate distance moved in this iteration and add to total
    // Formula: average number of turns of left and right * wheel circumference
    distanceMoved += 
      (leftDriveMotor.position(turns) * 200 + 
       rightDriveMotor.position(turns) * 200) / 2;
  }

  // Stop motors when target distance is reached
  leftDriveMotor.stop();
  rightDriveMotor.stop();
}

void PDrive(float Kp, float target, float speed) {
  float distanceMoved = 0; // Track how far we've traveled
  count = 0;
  thread Check = thread(LaternalCheck);
  // Continue driving until we reach the target distance
  while (count < MaxCount) {
    leftDriveMotor.setPosition(0, turns);
    rightDriveMotor.setPosition(0, turns);

    // Calculate heading error: difference between target and current heading
    float error = target - BrainInertial.rotation(degrees);
    // Calculate proportional correction value
    float P = Kp * error;

    // Apply correction to motor speeds:
    // - Left motor: base speed + correction
    // - Right motor: base speed - correction
    // This causes the robot to turn toward the target heading
    leftDriveMotor.setVelocity(speed + P, percent);
    rightDriveMotor.setVelocity(speed - P, percent);
    leftDriveMotor.spin(forward);
    rightDriveMotor.spin(forward);

    wait(0.1, seconds);

    // Calculate distance moved in this iteration and add to total
    // Formula: average number of turns of left and right * wheel circumference
    distanceMoved += 
      (leftDriveMotor.position(turns) * 200 + 
       rightDriveMotor.position(turns) * 200) / 2;
       
    Dist = distanceMoved;
  }

  // Stop motors when target distance is reached
  leftDriveMotor.stop();
  rightDriveMotor.stop();
}

// ========== PROPORTIONAL ROTATE FUNCTION ==========
/**
 * Rotates the robot to a specific target angle using proportional control
 * Kp - Proportional gain constant for rotation
 * target - Target angle to rotate to (relative to current position)
 * speed - Maximum rotation speed percentage (0-100%)
 */
void PRotate(float Kp, float target, float speed) {
  // Calculate absolute target rotation (current angle + desired change)
  float targetRotation = BrainInertial.rotation(degrees) + target;

  // Continue rotating until within ±0.1 degrees of tolerance range of target
  while 
    (abs(targetRotation - BrainInertial.rotation(degrees)) > 0.5){
    // leftDriveMotor.setPosition(0, turns);
    // rightDriveMotor.setPosition(0, turns);

    // Calculate error: difference between target and current rotation
    float error = targetRotation - BrainInertial.rotation(degrees);
    // Calculate proportional correction value
    float P = Kp * error;

    // Set motors for rotation:
    // - Positive target: spin clockwise
    // - Negative target: spin anticlockwise
    leftDriveMotor.setVelocity(P, percent);
    rightDriveMotor.setVelocity(-P, percent);
    leftDriveMotor.spin(forward);
    rightDriveMotor.spin(forward);

    wait(20, msec);
  }

  // Stop motors when target rotation is achieved
  leftDriveMotor.stop();
  rightDriveMotor.stop();
}


void Path() {
  roller.setMaxTorque(100, percent);
  roller.setVelocity(100, percent);
  roller.spin(forward);

  PDrive(Kp_DRIVE, 0, 2275, MAX_SPEED);
  PRotate(Kp_ROTATE, -90, MAX_SPEED);
  PDrive(Kp_DRIVE, -90, 1900, MAX_SPEED);

  PRotate(Kp_ROTATE, 45, MAX_SPEED);
  PDrive(Kp_DRIVE, -45, 450, MAX_SPEED);
  PRotate(Kp_ROTATE, -15, MAX_SPEED);
  PDrive(Kp_DRIVE, -60, 300, MAX_SPEED);
  // leftDriveMotor.setVelocity(100, percent);
  // rightDriveMotor.setVelocity(100, percent);
  // leftDriveMotor.spin(forward);
  // rightDriveMotor.spin(forward);
  // PRotate(Kp_ROTATE, -90, MAX_SPEED);

  // timer stopTimer;
  // stopTimer.clear(); // Reset timer to 0
  
  // // Stop only the base motors
  // leftDriveMotor.stop();
  // rightDriveMotor.stop();
  
  // // Continue checking time without blocking other code
  // while (stopTimer.time() < 10000) {
  //     roller.setMaxTorque(100, percent);
  //     roller.setVelocity(100, percent);
  //     roller.spin(forward);
  // }

  // // PRotate(Kp_ROTATE, 180, MAX_SPEED);
  // PDrive(Kp_DRIVE, 180, 1500, MAX_SPEED);

  // roller.setMaxTorque(100, percent);
  // roller.setVelocity(-100, percent);
  // roller.spin(forward);

  // // PDrive(Kp_DRIVE, 180, 1000, MAX_SPEED);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Path();
}