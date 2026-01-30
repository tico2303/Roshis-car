// I2CBus.h
#pragma once
#include <Arduino.h>
#include <Wire.h>

class I2CBus {
public:
  I2CBus(uint8_t busNum = 0) : _busNum(busNum) {}

  bool begin(int sda, int scl, uint32_t freqHz = 400000) {
    _wire = (_busNum == 0) ? &Wire : &_wireAlt;

    _sda = sda;
    _scl = scl;
    _freqHz = freqHz;

    _wire->begin(_sda, _scl, _freqHz);
    return true;
  }

  TwoWire& wire() { return *_wire; }

  int sda() const { return _sda; }
  int scl() const { return _scl; }
  uint32_t freq() const { return _freqHz; }

  // Useful utility: scan the bus
  void scan(Stream& out = Serial) {
    out.println("I2C scan:");
    for (uint8_t addr = 1; addr < 127; addr++) {
      _wire->beginTransmission(addr);
      if (_wire->endTransmission() == 0) {
        out.print("  found 0x");
        if (addr < 16) out.print("0");
        out.println(addr, HEX);
      }
    }
  }

private:
  uint8_t _busNum;
  TwoWire* _wire = nullptr;

  // If busNum != 0,use an alternate TwoWire instance
  TwoWire _wireAlt = TwoWire(1);

  int _sda = -1;
  int _scl = -1;
  uint32_t _freqHz = 400000;
};