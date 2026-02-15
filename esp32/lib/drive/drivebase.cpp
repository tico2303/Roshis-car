#include "drivebase.h"

void DriveBase::update(uint32_t now_ms, float dt_sec) {
  const uint32_t timeout = _cmd.timeout_ms ? _cmd.timeout_ms : 250;

  if (now_ms - _lastCmdMs > timeout) {
    _left.stop();
    _right.stop();
  } else {
    _left.setU(_cmd.left_u);
    _right.setU(_cmd.right_u);
  }

  _left.update(dt_sec);
  _right.update(dt_sec);

  _tel.left = _left.telemetry();
  _tel.right = _right.telemetry();
  _tel.t_ms = now_ms;
}