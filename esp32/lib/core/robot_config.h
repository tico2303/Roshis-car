#pragma once
#include <Arduino.h>
#include "drivetrain.h"
#include "configs.h"
#include <Wire.h>
/*
====================== ESP32 PIN MAP (DARO) ======================

I2C BUS
- SDA : GPIO 21
- SCL : GPIO 22

DRIVETRAIN (L9110)
- Front Left  : GPIO 33 (A), 32 (B)
- Front Right : GPIO 19 (A), 18 (B)
- Rear Left   : GPIO 27 (A), 14 (B)
- Rear Right  : GPIO 25 (A), 26 (B)

SENSORS
- ToF VL53L0X
  - XSHUT : GPIO 35 is input-only
  - I2C   : SDA 21 / SCL 22

- Bumper Switch
  - GPIO 15 (active-low, INPUT_PULLUP)

ACTUATORS
- Feeder Servo
  - GPIO 16 (PWM)

- LED Ring (WS2812)
  - GPIO 23 (DATA)
  - Count: 32

NOTE
- GPIO 34–39 are input-only (OK sensors, not outputs)

===============================================================
*/
namespace RobotConfig {

  // -------------------- Serial --------------------
  extern const int SERIAL_BUAD_RATE;

  // -------------------- Drivetrain --------------------
  extern const uint8_t FL_A;
  extern const uint8_t FL_B;
  extern const uint8_t FR_A;
  extern const uint8_t FR_B;
  extern const uint8_t RL_A;
  extern const uint8_t RL_B;
  extern const uint8_t RR_A;
  extern const uint8_t RR_B;

  extern const bool INVERT_FL;
  extern const bool INVERT_FR;
  extern const bool INVERT_RL;
  extern const bool INVERT_RR;

  extern const int MAX_SPEED;
  extern const int DEADBAND;
  extern const uint32_t PWM_FREQ_HZ;

  DriveTrain::DriveTrainPins driveTrainPins();

  // -------------------- Protocol --------------------
  extern const int THR_SIGN;
  extern const int STR_SIGN;

  // -------------------- Bumper --------------------
  extern const gpio_num_t BUMPER_PIN;
  extern const bool BUMPER_ACTIVE_LOW;

   // -------------------- I2C Bus --------------------
  struct I2CBusConfig {
    TwoWire* wire;
    uint8_t sda;
    uint8_t scl;
    uint32_t hz;
    bool hasPins;   // ESP32: true = use begin(sda,scl)

    I2CBusConfig(TwoWire* w = &Wire,
               uint8_t sda_ = 21,
               uint8_t scl_ = 22,
               uint32_t hz_ = 100000,
               bool hasPins_ = true)
  : wire(w), sda(sda_), scl(scl_), hz(hz_), hasPins(hasPins_) {}
  };

  extern const I2CBusConfig I2C;

  // -------------------- TOF --------------------
  extern const TofConfig TOF_CFG[];
  extern const size_t TOF_CFG_COUNT;

  extern const bool TOF_AUTORECOVERY;
  extern const int  TOF_RECOVERY_THRESHOLD;


  // -------------------- LEDs --------------------
  extern const int NUM_LEDS;
  extern const int LED_PIN;

  // -------------------- Feeder --------------------
  extern const FeederConfig feederConfig;

}