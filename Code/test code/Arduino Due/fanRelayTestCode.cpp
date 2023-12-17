// Arduino code to toggle pin 31 with a 1-second interval

// Constants
const int pin = 31; // Pin number to toggle
const int interval = 1000; // Interval for toggling (1000 milliseconds = 1 second)

// Setup function
void setup() {
  pinMode(pin, OUTPUT); // Set pin 31 as an output
}

// Main loop function
void loop() {
  digitalWrite(pin, HIGH); // Set pin 31 to HIGH (enable)
  delay(interval); // Wait for the specified interval (1 second)
  digitalWrite(pin, LOW); // Set pin 31 to LOW (disable)
  delay(interval); // Wait for the specified interval (1 second)
}
