// tof_manager.cpp  (UPDATED)
#include "tof_manager.h"

TofManager::TofManager(const TofConfig* cfg, size_t count)
: _cfg(cfg), _count(count) {}

void TofManager::allShutdown() {
  for (size_t i = 0; i < _count; i++) {
    pinMode(_cfg[i].xshutPin, OUTPUT);
    digitalWrite(_cfg[i].xshutPin, LOW);
  }
}

bool TofManager::begin(TwoWire& wire) {
  _wire = &wire;

  // Allocate slots once
  if (_slots == nullptr) {
    _slots = new Slot[_count];
  }

  // Copy configs into slots
  for (size_t i = 0; i < _count; i++) {
    _slots[i].cfg = _cfg[i];
    _slots[i].ok = false;
  }

  // Reset all sensors, then init sequentially so we can assign unique addresses
  allShutdown();
  delay(10);

  for (size_t i = 0; i < _count; i++) {
    _slots[i].ok = initSlot(i);
  }

  _nextDueMs = millis();
  _rr = 0;
  _consecutiveErrors = 0;

  // true if at least one sensor came up
  for (size_t i = 0; i < _count; i++) {
    if (_slots[i].ok) return true;
  }
  return false;
}

bool TofManager::initSlot(size_t i) {
  Slot& s = _slots[i];

  // Enable this sensor
  digitalWrite(s.cfg.xshutPin, HIGH);
  delay(10);

  s.sensor.setTimeout(50);

  // init() expects device at default address
  if (!s.sensor.init()) {
    return false;
  }

  // Assign unique address so other sensors can coexist on the bus
  s.sensor.setAddress(s.cfg.addr);

  // Start continuous ranging
  s.sensor.startContinuous();
  return true;
}

void TofManager::busClear() {
  if (!_hasPins) return;

  // Pulse SCL until SDA releases (up to 9 pulses). Helps if SDA is stuck low.
  pinMode(_sdaPin, INPUT_PULLUP);
  pinMode(_sclPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(_sclPin, HIGH);
  delayMicroseconds(5);

  for (int i = 0; i < 9; i++) {
    if (digitalRead(_sdaPin) == HIGH) break;
    digitalWrite(_sclPin, LOW);
    delayMicroseconds(5);
    digitalWrite(_sclPin, HIGH);
    delayMicroseconds(5);
  }
}

bool TofManager::recover() {
  uint32_t now = millis();
  if (now - _lastRecoverMs < _recoverCooldownMs) {
    return false; // cooldown: avoid thrashing
  }
  _lastRecoverMs = now;

  // Put sensors in reset so they stop holding the bus
  allShutdown();
  delay(10);

  if (_wire) {
    _wire->end();
    delay(20);
  }

  // Attempt to clear a stuck bus (SDA held low)
  busClear();

  // Restart I2C
  if (_wire) {
    if (_hasPins) {
      _wire->begin(_sdaPin, _sclPin);
    } else {
      _wire->begin();
    }
    _wire->setClock(_i2cHz);
  }
  delay(20);

  // Re-init sensors (reassign addresses)
  bool ok = begin(*_wire);
  return ok;
}

bool TofManager::poll(TofReading& out) {
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

    out.id = (uint8_t)idx;
    out.ts_ms = now;

    if (timeout) {
      out.mm = -1;
      out.status = TOF_STATUS_TIMEOUT;
      _consecutiveErrors++;
    }
    else if (mm == 0xFFFF) { //treat sentinel as invalid
      out.mm = -1;
      out.status = TOF_STATUS_INVALID;
      _consecutiveErrors++;
    }
    else {
      out.mm = (int32_t)mm;
      out.status = TOF_STATUS_OK;
      _consecutiveErrors = 0;
    }

    if (_autoRecover && _consecutiveErrors >= _recoverThreshold) {
      _consecutiveErrors = 0;
      recover();
    }

    return true;
  }

  return false;
}