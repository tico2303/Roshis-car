#pragma once
#include <Arduino.h>
#include <VL53L0X.h>
#include "configs.h"
#include "i2c_bus.h"
#include "i2c_sensor.h"

struct TofReading {
  uint8_t id;        // index in config array
  int32_t mm;        // distance in mm, -1 if invalid
  uint8_t status;    // 0 ok, nonzero = error
  uint32_t ts_ms;    // millis()
};

enum : uint8_t {
  TOF_STATUS_OK        = 0,
  TOF_STATUS_TIMEOUT   = 1,
  TOF_STATUS_INVALID   = 2,
  TOF_STATUS_INIT_FAIL = 3,
};

class Protocol; // forward decl

class TofManager : public I2CSensor {
public:
  TofManager(const TofConfig* cfg, size_t count);

  // --- I2CSensor interface ---
  const char* name() const override { return "tof"; }
  bool begin(I2CBus& bus) override;
  void end() override;
  bool poll() override;
  bool ok() const override;

  bool publish(Protocol& proto) override;

  // --- Tof-specific controls ---
  void setPeriodMs(uint32_t ms) { _periodMs = ms; }
  void setAutoRecover(bool enabled) { _autoRecover = enabled; }
  void setRecoverThreshold(uint8_t consecutiveErrors) { _recoverThreshold = consecutiveErrors; }

  // Access latest reading (optional helper)
  bool hasReading() const { return _hasLast; }
  const TofReading& last() const { return _last; }

private:
  struct Slot {
    TofConfig cfg;
    VL53L0X sensor;
    bool ok = false;
  };

  void allShutdown();
  bool initSlot(size_t i);
  bool recover_();

  const TofConfig* _cfg = nullptr;
  size_t _count = 0;

  Slot* _slots = nullptr;
  I2CBus* _bus = nullptr;

  uint32_t _periodMs = 50;
  uint32_t _nextDueMs = 0;
  size_t _rr = 0;

  bool _autoRecover = true;
  uint8_t _recoverThreshold = 10;
  uint8_t _consecutiveErrors = 0;
  uint32_t _lastRecoverMs = 0;
  uint32_t _recoverCooldownMs = 2000;

  // Latest reading cache
  TofReading _last{};
  bool _hasLast = false;
  bool _lastPublished = true; // start "published" so we don't publish garbage
};