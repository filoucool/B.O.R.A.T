#include <Adafruit_NeoPixel.h>

#define PIN        6     // NeoPixel data pin
#define NUM_LEDS   8     // Number of LEDs in the stick
#define BRIGHTNESS 50    // Brightness level (0-255)
#define DELAY_TIME 100   // Delay between LED changes in milliseconds

// Initialize NeoPixel strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();           // Initialize the NeoPixel strip
  strip.setBrightness(BRIGHTNESS);
  strip.show();            // Clear all pixels (turn them off)
}

void loop() {
  // Test all LEDs with red, green, and blue colors
  testLEDs(strip.Color(255, 0, 0));  // Red
  testLEDs(strip.Color(0, 255, 0));  // Green
  testLEDs(strip.Color(0, 0, 255));  // Blue
}

void testLEDs(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);   // Set the color of the current LED
    strip.show();                    // Update the strip to display the new color
    delay(DELAY_TIME);               // Wait for the specified time
    strip.setPixelColor(i, 0);       // Turn off the current LED
  }
}
Converting the entire Arduino C++ code for NeoPixel control to Assembly language without using any libraries is a complex task and would result in a lengthy and hard-to-read Assembly code. However, I can give you some guidance on how to get started with this process.




