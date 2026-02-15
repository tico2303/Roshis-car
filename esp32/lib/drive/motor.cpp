#include "motor.h"
#include <Arduino.h>
#include <cmath>

static inline float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

bool Motor::begin(const MotorPins& pins,
                  const MotorPwmConfig& pwm,
                  bool invert_motor,
                  bool invert_encoder) {
  _pins = pins;
  _pwm = pwm;
  _invertMotor = invert_motor;

  // DRV8871 IN1/IN2
  pinMode(_pins.in1, OUTPUT);
  pinMode(_pins.in2, OUTPUT);
  // make sure its off at start
  digitalWrite(_pins.in1, LOW);
  digitalWrite(_pins.in2, LOW);

  // Setup two PWM channels (one per input)
  ledcSetup(_pwm.ch_in1, _pwm.freq_hz, _pwm.res_bits);
  ledcSetup(_pwm.ch_in2, _pwm.freq_hz, _pwm.res_bits);

  ledcAttachPin(_pins.in1, _pwm.ch_in1);
  ledcAttachPin(_pins.in2, _pwm.ch_in2);

  // Ensure stopped
  ledcWrite(_pwm.ch_in1, 0);
  ledcWrite(_pwm.ch_in2, 0);

  // Encoder: with voltage dividers / push-pull, avoid internal pulls
  ESP32Encoder::useInternalWeakPullResistors = puType::none;

  pinMode(_pins.encA, INPUT);
  pinMode(_pins.encB, INPUT);

  // HalfQuad for now
  if (invert_encoder) {
    _enc.attachHalfQuad(_pins.encB, _pins.encA);
  } else {
    _enc.attachHalfQuad(_pins.encA, _pins.encB);
  }

  _enc.clearCount();
  _lastCount = 0;
  _encAttached = true;
  return true;
}

void Motor::setU(float u_norm) {
  _cmd = clampf(u_norm, -1.0f, 1.0f);
}

void Motor::stop() {
  _cmd = 0.0f;
  apply(0.0f);
}

void Motor::apply(float u) {
  if (_invertMotor) u = -u;

  const uint32_t maxDuty = (1u << _pwm.res_bits) - 1;
  const uint32_t duty = (uint32_t)(fabsf(u) * (float)maxDuty);

  if (u > 0.0f) {
    // Forward: IN1 PWM, IN2 LOW
    ledcWrite(_pwm.ch_in1, duty);
    ledcWrite(_pwm.ch_in2, 0);
  } else if (u < 0.0f) {
    // Reverse: IN1 LOW, IN2 PWM
    ledcWrite(_pwm.ch_in1, 0);
    ledcWrite(_pwm.ch_in2, duty);
  } else {
    // Stop/coast: both LOW
    ledcWrite(_pwm.ch_in1, 0);
    ledcWrite(_pwm.ch_in2, 0);
  }
}

void Motor::update(float dt_sec) {
  apply(_cmd);

  if (!_encAttached || dt_sec <= 0.0f) {
    _tel = {};
    return;
  }

  const int64_t total = _enc.getCount();
  const int32_t dticks = (int32_t)(total - _lastCount);
  _lastCount = total;

  _tel.dticks = dticks;
  _tel.ticks_total = total;

  // rad/s (wheel) if configured
  if (_pulsesPerWheelRev > 0.0f) {
    const float ticks_per_sec = (float)dticks / dt_sec;
    const float wheel_rev_s = ticks_per_sec / _pulsesPerWheelRev;
    _tel.wheel_rad_s = wheel_rev_s * 2.0f * 3.1415926535f;
  } else {
    _tel.wheel_rad_s = 0.0f;
  }
}