// serial_health_test.cpp
//
// Minimal serial health diagnostic using COBS-framed protocol messages.
// Sends a numbered boot message once per second. Run on ESP32, then
// read on Pi with the COBS-aware ndjson_bridge, or use this snippet:
//
//   python3 -c "
//   import serial, time
//   from daro_ndjson_bridge.ndjson_bridge_node import _cobs_decode, _crc16
//   s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
//   s.reset_input_buffer()
//   buf = bytearray()
//   ok = bad = 0
//   while True:
//       chunk = s.read(s.in_waiting or 1)
//       if not chunk: continue
//       buf.extend(chunk)
//       while b'\x00' in buf:
//           idx = buf.index(b'\x00')
//           raw = bytes(buf[:idx]); del buf[:idx+1]
//           decoded = _cobs_decode(raw)
//           if decoded:
//               ok += 1
//           else:
//               bad += 1
//           if (ok+bad) % 10 == 0:
//               print(f'ok={ok} bad={bad} rate={100*bad/(ok+bad):.1f}%')
//   "
//
// If you see >1% bad at 1 msg/sec, your cable or CP2102 is faulty.

#include <Arduino.h>
#include "robot_config.h"
#include "protocol.h"

static Protocol proto(Serial);
static uint32_t seq = 0;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(1000);
  proto.sendBoot(0);
}

void loop() {
  seq++;
  proto.sendBoot(seq);
  delay(1000);
}
