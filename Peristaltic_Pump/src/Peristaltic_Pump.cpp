/*
 * Project myProject
 * Author: Your Name
 * Date:
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "neopixel.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

const int pumpOnePin = D8;
const int pumpTwoPin = D9;

const int ring1Pin = D2;
const int ring2Pin = D3;
const int ringPixelCount = 12;

// Two 12-pixel WS2812B NeoPixel rings.
Adafruit_NeoPixel ringOne(ringPixelCount, SPI1, WS2812B);
Adafruit_NeoPixel ringTwo(ringPixelCount, SPI, WS2812B);

unsigned long lastToggleTime = 0;
bool pumpOneState = false;

void updateRingPulse(Adafruit_NeoPixel &ring, bool isActive) {
  if (!isActive) {
    ring.clear();
    ring.show();
    return;
  }

  uint8_t brightness = (millis() / 20) % 512;
  if (brightness > 255) {
    brightness = 511 - brightness;
  }

  for (int i = 0; i < ring.numPixels(); ++i) {
    ring.setPixelColor(i, 0, 0, brightness);
  }

  ring.show();
}

// setup() runs once, when the device is first turned on
void setup() {
  pinMode(pumpOnePin, OUTPUT);
  pinMode(pumpTwoPin, OUTPUT);

  digitalWrite(pumpOnePin, LOW);
  digitalWrite(pumpTwoPin, LOW);

  ringOne.begin();
  ringOne.show();
  ringTwo.begin();
  ringTwo.show();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  if (millis() - lastToggleTime >= 10000) {
    lastToggleTime = millis();
    pumpOneState = !pumpOneState;

    digitalWrite(pumpOnePin, pumpOneState ? HIGH : LOW);
    digitalWrite(pumpTwoPin, pumpOneState ? LOW : HIGH);
  }

  updateRingPulse(ringOne, digitalRead(pumpOnePin) == HIGH);
  updateRingPulse(ringTwo, digitalRead(pumpTwoPin) == HIGH);
}