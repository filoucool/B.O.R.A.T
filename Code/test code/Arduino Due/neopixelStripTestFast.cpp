#include <Adafruit_NeoPixel.h>

#define PIN A0
#define NUM_LEDS 8
#define DELAY_TIME 50

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin(); // Initialize the NeoPixel strip
  strip.show();  // Ensure all LEDs are off
}

void loop() {
  // Strobe effect
  strobe(255, 0, 0, 5, DELAY_TIME); // Red
  fade(0, 255, 0, 1, DELAY_TIME); // Green
  strobe(0, 0, 255, 5, DELAY_TIME); // Blue
  fade(255, 0, 0, 1, DELAY_TIME); // Red
  strobe(0, 255, 0, 5, DELAY_TIME); // Green
  fade(0, 0, 255, 1, DELAY_TIME); // Blue
}

void strobe(uint8_t r, uint8_t g, uint8_t b, int repetitions, int delayTime) {
  for (int j = 0; j < repetitions; j++) {
    setColor(r, g, b);
    delay(delayTime);
    setColor(0, 0, 0);
    delay(delayTime);
  }
}

void fade(uint8_t r, uint8_t g, uint8_t b, int repetitions, int delayTime) {
  for (int j = 0; j < repetitions; j++) {
    for (int k = 0; k < 256; k++) {
      setColor((r * k) / 255, (g * k) / 255, (b * k) / 255);
      delay(delayTime / 100);
    }
    for (int k = 255; k >= 0; k--) {
      setColor((r * k) / 255, (g * k) / 255, (b * k) / 255);
      delay(delayTime / 100);
    }
  }
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}
