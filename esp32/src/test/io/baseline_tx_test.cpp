// baseline_tx_test.cpp
//
// Bare-bones serial TX test. No COBS, no CRC, no protocol library.
// Sends two message types at realistic rates:
//   - "imu"  ~190 bytes every 100ms (10Hz) — matches real IMU payload size
//   - "enc"  ~120 bytes every 100ms (10Hz) — matches real encoder payload size
//
// Build: pio run -e test_baseline_tx -t upload
// Read:  python3 pi/test_baseline_rx.py /dev/ttyUSB0 115200

#include <Arduino.h>
#include "robot_config.h"

static uint32_t seq_imu = 0;
static uint32_t seq_enc = 0;

static uint32_t last_imu_ms = 0;
static uint32_t last_enc_ms = 0;

static constexpr uint32_t IMU_PERIOD_MS  = 100;  // 10Hz
static constexpr uint32_t ENC_PERIOD_MS  = 100;  // 10Hz

// Stagger: enc fires 50ms after imu to avoid back-to-back bursts
static constexpr uint32_t ENC_OFFSET_MS  = 50;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);

  // Flush any garbage
  while (Serial.available()) Serial.read();

  Serial.println("{\"type\":\"diag\",\"msg\":\"baseline_tx_test_start\"}");

  last_imu_ms = millis();
  last_enc_ms = millis() + ENC_OFFSET_MS;
}

void loop() {
  uint32_t now = millis();

  // ---- IMU message (~190 bytes) ----
  if (now - last_imu_ms >= IMU_PERIOD_MS) {
    last_imu_ms = now;
    seq_imu++;

    // Build a message the same size as the real IMU payload
    // Real: {"v":1,"type":"sens","seq":1234,"t_ms":56789,"data":{"sensor":"imu","linear_acceleration":{"x":-9.6175,"y":0.4663,"z":2.1997},"angular_velocity":{"x":-0.0023,"y":0.0014,"z":0.0089}}}
    Serial.print("{\"type\":\"imu\",\"seq\":");
    Serial.print(seq_imu);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"ax\":-9.6175,\"ay\":0.4663,\"az\":2.1997");
    Serial.print(",\"gx\":-0.0023,\"gy\":0.0014,\"gz\":0.0089");
    Serial.print(",\"pad\":\"AAAAAAAAAAAAAAAAAAAAAAAAAAA\"");  // pad to ~190 bytes
    Serial.println("}");
  }

  // ---- Encoder message (~120 bytes) ----
  if (now - last_enc_ms >= ENC_PERIOD_MS) {
    last_enc_ms = now;
    seq_enc++;

    // Real: {"type":"enc","seq":1,"t_ms":1234,"left":{"dt":5,"tot":12345,"rad_s":1.23},"right":{"dt":-3,"tot":-9876,"rad_s":-0.98}}
    Serial.print("{\"type\":\"enc\",\"seq\":");
    Serial.print(seq_enc);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"l_dt\":5,\"l_tot\":12345,\"l_rs\":1.2345");
    Serial.print(",\"r_dt\":-3,\"r_tot\":-9876,\"r_rs\":-0.9876");
    Serial.println("}");
  }
}
