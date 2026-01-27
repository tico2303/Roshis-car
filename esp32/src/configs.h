#pragma once
#include <Arduino.h>

//-------feeder config
struct FeederConfig {
    int pin;
    int closedDeg;
    int openDeg;
    int hz;
    int minUs;
    int maxUs;
  };
//-----TOF manager config
// One physical TOF sensor description
struct TofConfig {
  uint8_t xshutPin; // GPIO controlling XSHUT
  uint8_t addr;     // unique 7-bit I2C address (e.g. 0x30, 0x31, ...)
};
