#include <ros.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <stepper_motor_control/StepperData.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pins for the TB6600 drivers (replace these with your actual pin numbers)
const int stepPins[] = {2, 4, 6, 8, 10, 12};
const int dirPins[] = {3, 5, 7, 9, 11, 13};
#define safetyPin = 32;

const int numSteppers = 6;
AccelStepper steppers[numSteppers] = {
  AccelStepper(1, stepPins[0], dirPins[0]),
  AccelStepper(1, stepPins[1], dirPins[1]),
  AccelStepper(1, stepPins[2], dirPins[2]),
  AccelStepper(1, stepPins[3], dirPins[3]),
  AccelStepper(1, stepPins[4], dirPins[4]),
  AccelStepper(1, stepPins[5], dirPins[5]),
};

ros::NodeHandle nh;
stepper_motor_control::StepperData stepperData;


#define NEO_PIN 6
#define NEO_COUNT 8

Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
// NEO ENUM:
//  1: estop          5: motor status
//  2: temp warning   6: main joint motor alarm
//  3: fan status     7: brake 1 status
//  4: tool status    8: brake 2 status

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#define outputPin = 42

#define mAlarmPin = 52



void stepperDataCallback(const stepper_motor_control::StepperData &data) {
  if (data.speed.size() == numSteppers && data.acceleration.size() == numSteppers) {
    for (int i = 0; i < numSteppers; i++) {
      steppers[i].setMaxSpeed(data.speed[i]);
      steppers[i].setAcceleration(data.acceleration[i]);
    }
  }
}

ros::Subscriber<stepper_motor_control::StepperData> sub("stepper_data", stepperDataCallback);

// Sensor value publisher
std_msgs::Int32 sensorValueMsg;
ros::Publisher sensorValuePub("sensor_value", &sensorValueMsg);

// Tool communication
void toolDataCallback(const std_msgs::String &data) {
  Serial2.print(data.data);
}

ros::Subscriber<std_msgs::String> toolSub("tool", toolDataCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(toolSub);
  nh.advertise(sensorValuePub);

  for (int i = 0; i < numSteppers; i++) {
    steppers[i].setMaxSpeed(200);
    steppers[i].setAcceleration(100);
    steppers[i].moveTo(1000);
  }

  // Sensor serial setup
  Serial1.begin(9600); // Adjust the baud rate according to your sensor's specifications

  // Tool serial setup
  Serial2.begin(9600); // Adjust the baud rate according to your tool's specifications

  // Safety input pin setup
  pinMode(safetyPin, INPUT_PULLUP);

  //initialise the neopixels to off
  strip.begin();
  strip.show();

  //initialise the temperature sensor
  sensors.begin();
}

void loop() {
  for (int i = 0; i < numSteppers; i++) {
    steppers[i].run();
  }

  // Read sensor value from Serial1
  if (Serial1.available()) {
    int sensorValue = Serial1.parseInt(); // You can change the data type depending on your sensor
    sensorValueMsg.data = sensorValue;
    sensorValuePub.publish(&sensorValueMsg);
  }

  // Safety input check
  if (digitalRead(safetyPin) == LOW) {
    for (int i = 0; i < numSteppers; i++) {
      steppers[i].setMaxSpeed(0);
      steppers[i].setAcceleration(0);
    
    }
    strip.setPixelColor(1, strip.Color(255, 0, 0)); // Set all pixels to red
  }

  // Call sensors.requestTemperatures() to issue a global temperature request
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // If temperature is greater than or equal to 28 degrees Celsius, turn on the output pin
  if (tempC >= 28.0) {
    digitalWrite(outputPin, HIGH);
    strip.setPixelColor(2, strip.Color(255, 0, 0));
  } else {
    digitalWrite(outputPin, LOW);
    strip.setPixelColor(2, strip.Color(0, 255, 0));
  }

  // main joint motor alarm check
  if (digitalRead(mAlarmPin) == HIGH) {
    strip.setPixelColor(1, strip.Color(255, 0, 0));
  } else {
    strip.setPixelColor(1, strip.Color(0, 255, 0));
  }

  strip.show(); //update les neopixels
  nh.spinOnce();
}
