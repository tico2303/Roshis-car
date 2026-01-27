#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "robot_config.h"
#include "configs.h"
/**
 * Feeder
 * ------
 * Small, non-blocking servo controller for a "feeder door" mechanism.
 *
 * Responsibilities:
 *  - own/attach the servo
 *  - open/close positions (degrees)
 *  - optional auto-close timer
 *
 * Non-responsibilities:
 *  - protocol parsing
 *  - logging/Serial prints
 *  - business logic about "when" to feed
 */
/*
struct FeederConfig {
    int pin;
    int closedDeg;
    int openDeg;
    int hz;
    int minUs;
    int maxUs;
  };
  */
class Feeder {
public:
 
Feeder();
  explicit Feeder(const FeederConfig& cfg);
  void begin();
  void update();

  void open(uint32_t autoCloseMs = 700);  // autoCloseMs=0 => stay open
  void close();

  bool isOpen() const { return _isOpen; }

private:
  /*
  const FeederConfig _cfg = {
    22,    // pin
    10,    // closedDeg
    45,    // openDeg
    50,    // hz
    500,   // minUs
    2500   // maxUs
  };
  */
  const FeederConfig _cfg;
  Servo _servo;

  bool _isOpen = false;
  uint32_t _autoCloseAtMs = 0;

  void _writeDeg(int deg);
  int _channel =0;
};