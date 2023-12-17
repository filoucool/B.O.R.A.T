#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Set pin connections for TB6600 stepper drivers
const int motorPins[6][2] = {
  {2, 3},
  {4, 5},
  {6, 7},
  {8, 9},
  {10, 11},
  {12, 13}
};

// Create AccelStepper instances
AccelStepper stepper1(AccelStepper::DRIVER, motorPins[0][0], motorPins[0][1]);
AccelStepper stepper2(AccelStepper::DRIVER, motorPins[1][0], motorPins[1][1]);
AccelStepper stepper3(AccelStepper::DRIVER, motorPins[2][0], motorPins[2][1]);
AccelStepper stepper4(AccelStepper::DRIVER, motorPins[3][0], motorPins[3][1]);
AccelStepper stepper5(AccelStepper::DRIVER, motorPins[4][0], motorPins[4][1]);
AccelStepper stepper6(AccelStepper::DRIVER, motorPins[5][0], motorPins[5][1]);

// Array of stepper instances
AccelStepper steppers[] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};

// Set acceleration value for all motors
const int acceleration = 1200;

void setup() {
  // Set initial speed and acceleration for all motors
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(random(200, 2001));
    steppers[i].setAcceleration(acceleration);
  }

  // Seed random number generator
  randomSeed(analogRead(0));
}

void loop() {
  // Update the speed of each motor at random intervals
  for (int i = 0; i < 6; i++) {
    if (random(0, 100) < 40) { // 40% chance of changing speed
      steppers[i].setMaxSpeed(random(200, 2001));
    }
    steppers[i].run();
  }
}
