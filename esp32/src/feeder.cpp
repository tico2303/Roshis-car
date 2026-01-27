#include "feeder.h"

Feeder::Feeder(const FeederConfig& cfg)
: _cfg(cfg) {}

void Feeder::begin() {
  Serial.println("[Feeder] begin()");

  _servo.setPeriodHertz(_cfg.hz);

  // Pick a channel explicitly (0..15 usually)
  _channel = 7; // choose something not used elsewhere
  _servo.attach(_cfg.pin);

  bool ok = _servo.attached();
  Serial.print("[Feeder] attach pin=");
  Serial.print(_cfg.pin);
  Serial.print(" ch=");
  Serial.print(_channel);
  Serial.print(" ok=");
  Serial.println(ok ? "YES" : "NO");
  // start closed
  _isOpen = false;
  _autoCloseAtMs = 0;
  _writeDeg(_cfg.closedDeg);
}

void Feeder::update() {
  if (_autoCloseAtMs == 0) return;

  // signed subtraction handles millis() wraparound correctly
  if ((int32_t)(millis() - _autoCloseAtMs) >= 0) {
    Serial.println("[Feeder] auto-close");
    close();
  }
}

void Feeder::open(uint32_t autoCloseMs) {
  if (_isOpen) {
    // STRICT RULE: ignore presses while already open
    Serial.println("[Feeder] open ignored (already open)");
    return;
  }

  const uint32_t now = millis();

  Serial.print("[Feeder] open(ms=");
  Serial.print(autoCloseMs);
  Serial.println(")");

  _isOpen = true;
  _writeDeg(_cfg.openDeg);

  if (autoCloseMs == 0) {
    _autoCloseAtMs = 0;
    Serial.println("[Feeder] WARNING: opened with ms=0 (no auto-close)");
    return;
  }

  _autoCloseAtMs = now + autoCloseMs;

  Serial.print("[Feeder] scheduled close at t=");
  Serial.println(_autoCloseAtMs);
}

void Feeder::close() {
  Serial.println("[Feeder] close()");
  _isOpen = false;
  _autoCloseAtMs = 0;
  _writeDeg(_cfg.closedDeg);
}

void Feeder::_writeDeg(int deg) {
  // Safety clamp (servo.write() expects 0..180)
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;

  Serial.print("[Feeder] write(");
  Serial.print(deg);
  Serial.println(")");

  _servo.write(deg);
}