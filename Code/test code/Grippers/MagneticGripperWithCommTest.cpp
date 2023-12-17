#include <Wire.h>

#define SLAVE_ADDR 0x12

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write("o");
  Wire.endTransmission();
  delay(2000);

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write("c");
  Wire.endTransmission();
  delay(2000);
}
