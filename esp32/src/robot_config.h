#pragma once
#include <Arduino.h>
#include "drivetrain.h"
#include "configs.h"

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

  // -------------------- TOF --------------------
  extern const TofConfig TOF_CFG[];
  extern const size_t TOF_CFG_COUNT;

  extern const bool TOF_AUTORECOVERY;
  extern const int  TOF_RECOVERY_THRESHOLD;

  extern const int WIRE_CLOCK;
  extern const uint8_t I2C_SDA;
  extern const uint8_t I2C_SCL;

  // -------------------- LEDs --------------------
  extern const int NUM_LEDS;
  extern const int LED_PIN;

  // -------------------- Feeder --------------------
  extern const FeederConfig feederConfig;

}