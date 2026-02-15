#pragma once
#include <cstdint>
#include "motor.h"

struct DriveCommand {
  float left_u = 0.0f;
  float right_u = 0.0f;
  uint32_t t_ms = 0;
  uint32_t timeout_ms = 250;
};

struct DriveTelemetry {
  MotorTelemetry left;
  MotorTelemetry right;
  uint32_t t_ms = 0;
};

class DriveBase {
public:
  DriveBase(Motor& left, Motor& right) : _left(left), _right(right) {}

  void setCommand(const DriveCommand& cmd, uint32_t now_ms) {
    _cmd = cmd;
    _lastCmdMs = cmd.t_ms ? cmd.t_ms : now_ms;
  }

  void update(uint32_t now_ms, float dt_sec);

  DriveTelemetry telemetry() const { return _tel; }

private:
  Motor& _left;
  Motor& _right;

  DriveCommand _cmd{};
  uint32_t _lastCmdMs = 0;

  DriveTelemetry _tel{};
};