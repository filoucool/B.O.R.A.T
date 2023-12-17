#include <AccelStepper.h>

// Define stepper motors
AccelStepper stepper1(1, 2, 50); // Motor 1, Step pin 2, Direction pin 3
AccelStepper stepper2(1, 3, 48); // Motor 2, Step pin 4, Direction pin 5
AccelStepper stepper3(1, 4, 46); // Motor 3, Step pin 6, Direction pin 7
AccelStepper stepper4(1, 5, 44); // Motor 4, Step pin 8, Direction pin 9
AccelStepper stepper5(1, 6, 42); // Motor 5, Step pin 10, Direction pin 11
AccelStepper stepper6(1, 7, 40); // Motor 6, Step pin 12, Direction pin 13

// Array of pointers to stepper objects
AccelStepper* steppers[] = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};

// Constants
const int numMotors = 6;
const int acceleration = 500;

// Variables
unsigned long previousMillis = 0;
int interval = 0;

void setup() {
  randomSeed(analogRead(0));

  for (int i = 0; i < numMotors; i++) {
    steppers[i]->setMaxSpeed(2000);
    steppers[i]->setSpeed(500);
    steppers[i]->setAcceleration(acceleration);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if the interval has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Choose a random motor
    int randomMotor = random(0, numMotors);

    // Set random speed
    int randomSpeed = random(500, 2001);
    steppers[randomMotor]->setSpeed(randomSpeed);

    // Set the next random interval
    interval = random(200, 1501);
  }

  // Run all stepper motors
  for (int i = 0; i < numMotors; i++) {
    steppers[i]->runSpeed();
  }
}
