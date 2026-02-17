#pragma once
#include <Arduino.h>
#include <DFRobot_BMI160.h>
#include "i2c_bus.h"
#include "i2c_sensor.h"

struct Bmi160Reading {
  float ax, ay, az;   // m/s²
  float gx, gy, gz;   // rad/s
  uint32_t ts_ms;
};

class Bmi160Imu : public I2CSensor {
public:
  explicit Bmi160Imu(int8_t addr = 0x68) : _addr(addr) {}

  const char* name() const override { return "bmi160"; }

  bool begin(I2CBus& bus) override {
    _bus = &bus;

    if (_bmi.I2cInit(_addr) != BMI160_OK) {
      _ok = false;
      return false;
    }

    _ok = true;
    return true;
  }

  void end() override { _ok = false; }

  bool poll() override {
    if (!_ok) return false;

    uint32_t now = millis();
    if ((int32_t)(now - _nextPollMs) < 0) return false;
    _nextPollMs = now + _periodMs;

    int16_t accelGyro[6];
    if (_bmi.getAccelGyroData(accelGyro) != 0) return false;

    // Library returns raw 16-bit values
    // Default ranges: accel ±2g, gyro ±250 dps
    constexpr float ACC_SCALE = 9.80665f * 2.0f / 32768.0f;  // ±2g -> m/s²
    constexpr float GYR_SCALE = (250.0f / 32768.0f) * (PI / 180.0f); // ±250dps -> rad/s

    _last.gx = accelGyro[0] * GYR_SCALE;
    _last.gy = accelGyro[1] * GYR_SCALE;
    _last.gz = accelGyro[2] * GYR_SCALE;
    _last.ax = accelGyro[3] * ACC_SCALE;
    _last.ay = accelGyro[4] * ACC_SCALE;
    _last.az = accelGyro[5] * ACC_SCALE;
    _last.ts_ms = now;

    _hasNew = true;
    return true;
  }

  bool ok() const override { return _ok; }

  // Publish IMU data as NDJSON sensor packet.
  // Format matches ROS2 sensor_msgs/Imu field names.
  bool publish(Protocol& proto) override {
    if (!_hasNew) return false;
    _hasNew = false;

    JsonDocument doc;
    proto.sendSensorPacketBegin(doc, _last.ts_ms);

    JsonObject data = doc["data"].as<JsonObject>();
    data["sensor"] = "imu";

    // Linear acceleration (m/s²) - maps to sensor_msgs/Imu.linear_acceleration
    JsonObject lin = data["linear_acceleration"].to<JsonObject>();
    lin["x"] = serialized(String(_last.ax, 4));
    lin["y"] = serialized(String(_last.ay, 4));
    lin["z"] = serialized(String(_last.az, 4));

    // Angular velocity (rad/s) - maps to sensor_msgs/Imu.angular_velocity
    JsonObject ang = data["angular_velocity"].to<JsonObject>();
    ang["x"] = serialized(String(_last.gx, 4));
    ang["y"] = serialized(String(_last.gy, 4));
    ang["z"] = serialized(String(_last.gz, 4));

    proto.sendSensorPacketEnd(doc);
    return true;
  }

  void setPeriodMs(uint32_t ms) { _periodMs = ms; }
  const Bmi160Reading& last() const { return _last; }
  bool hasNew() const { return _hasNew; }

private:
  int8_t _addr;
  I2CBus* _bus = nullptr;
  DFRobot_BMI160 _bmi;
  bool _ok = false;
  bool _hasNew = false;

  uint32_t _periodMs = 50;   // 20Hz default (fits 115200 baud budget)
  uint32_t _nextPollMs = 0;

  Bmi160Reading _last{};
};
