//tof_manager.cpp
#include "tof_manager.h"
#include "protocol.h"  // for Protocol::sendTof

TofManager::TofManager(const TofConfig* cfg, size_t count)
: _cfg(cfg), _count(count) {}

void TofManager::allShutdown() {
  for (size_t i = 0; i < _count; i++) {
    pinMode(_cfg[i].xshutPin, OUTPUT);
    digitalWrite(_cfg[i].xshutPin, LOW);
  }
}

bool TofManager::begin(I2CBus& bus) {
  _bus = &bus;

  if (_slots == nullptr) {
    _slots = new Slot[_count];
  }

  for (size_t i = 0; i < _count; i++) {
    _slots[i].cfg = _cfg[i];
    _slots[i].ok = false;
  }

  _bus->begin();

  allShutdown();
  delay(10);

  for (size_t i = 0; i < _count; i++) {
    _slots[i].ok = initSlot(i);
  }

  _nextDueMs = millis();
  _rr = 0;
  _consecutiveErrors = 0;

  _hasLast = false;
  _lastPublished = true;

  return ok();
}

void TofManager::end() {
  allShutdown();
  // Note: We do NOT end() the I2C bus here; bus is shared by many sensors.
}

bool TofManager::ok() const {
  if (_count == 0 || _slots == nullptr) return false;
  for (size_t i = 0; i < _count; i++) {
    if (_slots[i].ok) return true;
  }
  return false;
}

bool TofManager::initSlot(size_t i) {
  Slot& s = _slots[i];

  digitalWrite(s.cfg.xshutPin, HIGH);
  delay(10);

  s.sensor.setTimeout(50);

  // If your VL53L0X library supports injecting TwoWire, do it here.
  // Otherwise it uses global Wire which your I2CBus config has already set up.
  if (!s.sensor.init()) {
    return false;
  }

  s.sensor.setAddress(s.cfg.addr);
  s.sensor.startContinuous();
  return true;
}

bool TofManager::recover_() {
  if (!_bus) return false;

  uint32_t now = millis();
  if (now - _lastRecoverMs < _recoverCooldownMs) return false;
  _lastRecoverMs = now;

  allShutdown();
  delay(10);

  _bus->recover();
  delay(20);

  // Re-init sensors and reassign addresses
  return begin(*_bus);
}

bool TofManager::poll() {
  if (!ok()) return false;

  uint32_t now = millis();
  if ((int32_t)(now - _nextDueMs) < 0) return false;
  _nextDueMs = now + _periodMs;

  for (size_t attempt = 0; attempt < _count; attempt++) {
    size_t idx = _rr % _count;
    _rr = (idx + 1) % _count;

    Slot& s = _slots[idx];
    if (!s.ok) continue;

    uint16_t mm = s.sensor.readRangeContinuousMillimeters();
    bool timeout = s.sensor.timeoutOccurred();

    _last.id = (uint8_t)idx;
    _last.ts_ms = now;

    if (timeout) {
      _last.mm = -1;
      _last.status = TOF_STATUS_TIMEOUT;
      _consecutiveErrors++;
    } else if (mm == 0xFFFF) {
      _last.mm = -1;
      _last.status = TOF_STATUS_INVALID;
      _consecutiveErrors++;
    } else {
      _last.mm = (int32_t)mm;
      _last.status = TOF_STATUS_OK;
      _consecutiveErrors = 0;
    }

    _hasLast = true;
    _lastPublished = false;

    if (_autoRecover && _consecutiveErrors >= _recoverThreshold) {
      _consecutiveErrors = 0;
      recover_();
    }

    return true;
  }

  return false;
}

bool TofManager::publish(Protocol& proto) {
  if (!_hasLast) return false;
  if (_lastPublished) return false;

  proto.sendTof(_last);
  _lastPublished = true;
  return true;
}