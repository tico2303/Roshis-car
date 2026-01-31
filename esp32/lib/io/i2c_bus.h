//i2c_bus.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "robot_config.h"
class I2CBus {
public:


  explicit I2CBus(const RobotConfig::I2CBusConfig& cfg) : _cfg(cfg) {}

  bool begin();
  void end();

  TwoWire& wire() { return *_cfg.wire; }
  uint32_t hz() const { return _cfg.hz; }

  void setClock(uint32_t hz);

  // Debug / bring-up helper
  void scan(Stream& out);

  // Recovery helpers
  bool clearStuckBus();  // SCL pulses if SDA stuck low
  bool recover();        // end(), clearStuckBus(), begin()

private:
  RobotConfig::I2CBusConfig _cfg;

  void sclPulseClear_(); // internal helper
};