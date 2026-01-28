// isensor.h
#pragma once
#include <Arduino.h>
#include <Wire.h>

class ISensor {
public:
  virtual ~ISensor() = default;
  virtual const char* name() const = 0;
  virtual bool begin(TwoWire& wire) = 0;
  virtual bool update() = 0;     // poll / read
};