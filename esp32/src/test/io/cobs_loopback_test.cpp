// cobs_loopback_test.cpp
//
// Self-test: encode a known JSON+CRC payload with COBS, decode it,
// verify round-trip. Then send COBS frames over serial at 1Hz so Pi
// can verify with test_serial_raw.py.
//
// Also dumps the raw COBS bytes as hex so you can compare what the
// ESP32 thinks it sent vs what the Pi receives.
//
// Build: pio run -e test_cobs_loopback -t upload
// Read:  python3 pi/test_serial_raw.py /dev/ttyUSB0 115200

#include <Arduino.h>
#include "robot_config.h"
#include "protocol.h"

static Protocol proto(Serial);

// Expose cobs_encode/cobs_decode for local testing by duplicating them.
// (They are static in protocol.cpp, so we duplicate here for the self-test.)

static uint16_t test_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

static size_t test_cobs_encode(const uint8_t* in, size_t in_len, uint8_t* out) {
  size_t out_idx = 0;
  size_t code_idx = out_idx++;
  uint8_t code = 1;
  for (size_t i = 0; i < in_len; i++) {
    if (in[i] != 0x00) {
      out[out_idx++] = in[i];
      if (++code == 0xFF) {
        out[code_idx] = code;
        code_idx = out_idx++;
        code = 1;
      }
    } else {
      out[code_idx] = code;
      code_idx = out_idx++;
      code = 1;
    }
  }
  out[code_idx] = code;
  out[out_idx++] = 0x00;
  return out_idx;
}

static size_t test_cobs_decode(const uint8_t* in, size_t in_len, uint8_t* out) {
  size_t out_idx = 0;
  size_t i = 0;
  while (i < in_len) {
    uint8_t code = in[i++];
    if (code == 0x00) return 0;
    for (uint8_t j = 1; j < code; j++) {
      if (i >= in_len) return 0;
      out[out_idx++] = in[i++];
    }
    if (code < 0xFF && i < in_len) {
      out[out_idx++] = 0x00;
    }
  }
  return out_idx;
}

static uint32_t seq = 0;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(1000);

  // --- Self-test: COBS round-trip ---
  const char* test_json = "{\"type\":\"test\",\"v\":1}";
  size_t json_len = strlen(test_json);

  // Build payload: json + \t + 4-hex-crc
  char payload[128];
  memcpy(payload, test_json, json_len);
  uint16_t c = test_crc16((const uint8_t*)test_json, json_len);
  payload[json_len] = '\t';
  payload[json_len + 1] = "0123456789abcdef"[(c >> 12) & 0x0f];
  payload[json_len + 2] = "0123456789abcdef"[(c >>  8) & 0x0f];
  payload[json_len + 3] = "0123456789abcdef"[(c >>  4) & 0x0f];
  payload[json_len + 4] = "0123456789abcdef"[ c        & 0x0f];
  size_t payload_len = json_len + 5;

  // Encode
  uint8_t encoded[140];
  size_t enc_len = test_cobs_encode((const uint8_t*)payload, payload_len, encoded);

  // Decode (skip trailing 0x00)
  uint8_t decoded[140];
  size_t dec_len = test_cobs_decode(encoded, enc_len - 1, decoded);

  // Verify
  bool pass = (dec_len == payload_len) && (memcmp(decoded, payload, payload_len) == 0);

  // Send self-test result as first COBS frame via proto
  if (pass) {
    proto.sendErr("debug", 0, "COBS self-test PASSED");
  } else {
    proto.sendErr("debug", 0, "COBS self-test FAILED");
  }
}

void loop() {
  seq++;

  // Send a known message through the real Protocol path
  proto.sendBoot(seq);

  delay(1000);
}
