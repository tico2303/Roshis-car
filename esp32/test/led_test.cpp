#include "led_controller.h"

const int NUM_LEDS = 32;
const int LED_PIN  = 23;

LedController led(LED_PIN, NUM_LEDS);

void setup() {
  Serial.begin(115200);

  // calibrate topIndex once; set it here.
  led.setEyeConfig(16, /*topIndex=*/0);
  // Optional styling
  led.setEyeColors(CRGB::Black, CRGB::Green);
  led.lookAt(0);
  led.begin();
  led.idle();
}

void loop() {
    //led.debugTopIndex(32);
    //led.debugShowIndex(16, 0);
  // Always run this so animations can progress
  led.update();

  // Demo state machine: change mode on a timer
  static unsigned long lastChange = 0;
  static int stage = 0;

 static unsigned long last = 0;
  static uint8_t pos = 0;
  unsigned long now = millis();
  if (now - lastChange >= 2000) {   // change every 2 seconds
    lastChange = now;

    stage = (stage + 1) % 2;
    switch (stage) {
      case 0: led.lookAt(12);   break;
      case 1: led.lookAt(4);   break;
      // case 2: led.thinking(); break;
      //case 3: led.idle();    break;
    }
  }

}