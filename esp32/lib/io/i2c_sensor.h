#pragma once
#include <Arduino.h>

class I2CBus;     
class Protocol;   

class I2CSensor {
public:
  virtual ~I2CSensor() = default;

  // friendly ID for logs/debug
  virtual const char* name() const = 0;

  // Init hardware, configure the bus, etc.
  virtual bool begin(I2CBus& bus) = 0;

  // cleanup/reset
  virtual void end() {}

  // Called frequently; returns true if new data became available
  virtual bool poll() = 0;

  // Health/availability
  virtual bool ok() const = 0;

  // Optional: publish latest data via Protocol (sensor decides format)
  // Return true if something was published.
  virtual bool publish(Protocol& proto) { (void)proto; return false; }
};