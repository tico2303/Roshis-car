#pragma once
#include <cstdint>
#include <Arduino.h>
#include <ESP32Encoder.h>

struct MotorPins {
  int in1;      // DRV8871 IN1
  int in2;      // DRV8871 IN2
  int encA;
  int encB;
};

struct MotorPwmConfig {
  int ch_in1;   // LEDC channel for IN1
  int ch_in2;   // LEDC channel for IN2
  int freq_hz;
  int res_bits;
};

struct MotorTelemetry {
  int32_t dticks = 0;
  int64_t ticks_total = 0;
  float   wheel_rad_s = 0.0f;
};

class Motor {
public:
  bool begin(const MotorPins& pins,
             const MotorPwmConfig& pwm,
             bool invert_motor = false,
             bool invert_encoder = false);

  // Open-loop command: -1..+1
  void setU(float u_norm);

  // Immediately stop output
  void stop();

  // Call at a fixed period (e.g., 20ms)
  void update(float dt_sec);

  MotorTelemetry telemetry() const { return _tel; }

  // Encoder helpers
  int64_t count() { return _enc.getCount(); }
  void resetCount() { _enc.clearCount(); _lastCount = 0; _tel = {}; }

  // Config knobs for later (PID etc.)
  void setPulsesPerWheelRev(float v) { _pulsesPerWheelRev = v; }

private:
  void apply(float u_norm);

  MotorPins _pins{};
  MotorPwmConfig _pwm{};
  bool _invertMotor = false;

  ESP32Encoder _enc;
  bool _encAttached = false;
  int64_t _lastCount = 0;

  float _cmd = 0.0f;                // -1..1
  float _pulsesPerWheelRev = 1.0f;  // set via config

  MotorTelemetry _tel;
};