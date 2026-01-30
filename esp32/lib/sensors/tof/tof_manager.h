// tof_manager.h  (UPDATED)
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "configs.h"

struct TofReading {
  uint8_t id;        // index in config array
  int32_t mm;        // distance in mm, -1 if invalid
  uint8_t status;    // 0 ok, nonzero = error
  uint32_t ts_ms;    // millis()
};


// named status codes (keeps callers from using magic numbers)
enum : uint8_t {
  TOF_STATUS_OK      = 0,
  TOF_STATUS_TIMEOUT = 1,
  TOF_STATUS_INVALID = 2,  // e.g. mm == 0xFFFF
};

class TofManager {
public:
  TofManager(const TofConfig* cfg, size_t count);

  bool begin(TwoWire& wire = Wire);
  bool poll(TofReading& out);

  void setPeriodMs(uint32_t ms) { _periodMs = ms; }
  size_t count() const { return _count; }

  // I2C recovery configuration (ESP32 Arduino Wire supports begin(sda,scl))
  void setI2CPins(uint8_t sdaPin, uint8_t sclPin) { _sdaPin = sdaPin; _sclPin = sclPin; _hasPins = true; }
  void setI2CHz(uint32_t hz) { _i2cHz = hz; }

  // Recovery controls
  void setAutoRecover(bool enabled) { _autoRecover = enabled; }
  void setRecoverThreshold(uint8_t consecutiveErrors) { _recoverThreshold = consecutiveErrors; }
  bool recover();

private:
  struct Slot {
    TofConfig cfg;
    VL53L0X sensor;
    bool ok = false;
  };

  void allShutdown();
  bool initSlot(size_t i);
  void busClear();

  const TofConfig* _cfg = nullptr;
  size_t _count = 0;

  Slot* _slots = nullptr;
  TwoWire* _wire = nullptr;

  uint32_t _periodMs = 50;
  uint32_t _nextDueMs = 0;
  size_t _rr = 0;

  // Recovery state
  bool _hasPins = false;
  uint8_t _sdaPin = 21;
  uint8_t _sclPin = 22;
  uint32_t _i2cHz = 100000;

  bool _autoRecover = true;
  uint8_t _recoverThreshold = 10;
  uint8_t _consecutiveErrors = 0;
  uint32_t _lastRecoverMs = 0;
  uint32_t _recoverCooldownMs = 2000;
};