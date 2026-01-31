//i2c_bus.cpp
#include "i2c_bus.h"

bool I2CBus::begin() {
  if (!_cfg.wire) return false;

  if (_cfg.hasPins) {
    _cfg.wire->begin(_cfg.sda, _cfg.scl);
  } else {
    _cfg.wire->begin();
  }
  _cfg.wire->setClock(_cfg.hz);
  delay(10);
  return true;
}

void I2CBus::end() {
  if (_cfg.wire) {
    _cfg.wire->end();
    delay(5);
  }
}

void I2CBus::setClock(uint32_t hz) {
  _cfg.hz = hz;
  if (_cfg.wire) _cfg.wire->setClock(_cfg.hz);
}

void I2CBus::scan(Stream& out) {
  out.println("I2C scan...");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    _cfg.wire->beginTransmission(addr);
    uint8_t err = _cfg.wire->endTransmission();
    if (err == 0) {
      out.print("  found 0x");
      if (addr < 16) out.print('0');
      out.println(addr, HEX);
      found++;
    }
    delay(2);
  }

  out.print("I2C scan done. Found: ");
  out.println(found);
}

bool I2CBus::clearStuckBus() {
  if (!_cfg.hasPins) return false;

  // If SDA is stuck LOW, pulse SCL up to 9 times to release it.
  pinMode(_cfg.sda, INPUT_PULLUP);
  pinMode(_cfg.scl, OUTPUT_OPEN_DRAIN);
  digitalWrite(_cfg.scl, HIGH);
  delayMicroseconds(5);

  if (digitalRead(_cfg.sda) == HIGH) {
    return true; // not stuck
  }

  sclPulseClear_();

  // Re-check
  pinMode(_cfg.sda, INPUT_PULLUP);
  return (digitalRead(_cfg.sda) == HIGH);
}

void I2CBus::sclPulseClear_() {
  for (int i = 0; i < 9; i++) {
    digitalWrite(_cfg.scl, LOW);
    delayMicroseconds(5);
    digitalWrite(_cfg.scl, HIGH);
    delayMicroseconds(5);
  }
}

bool I2CBus::recover() {
  end();
  delay(20);
  clearStuckBus();
  delay(5);
  return begin();
}