#include "robot_config.h"
#include "config.h"

namespace RobotConfig {

  const int SERIAL_BUAD_RATE = 115200;

  // -------------------- Drivetrain --------------------
  const uint8_t FL_A = 33;
  const uint8_t FL_B = 32;

  const uint8_t FR_A = 19;
  const uint8_t FR_B = 18;

  const uint8_t RL_A = 27;
  const uint8_t RL_B = 14;

  const uint8_t RR_A = 25;
  const uint8_t RR_B = 26;

  const bool INVERT_FL = true;
  const bool INVERT_FR = true;

  const bool INVERT_RL = false;
  const bool INVERT_RR = false;

  const int MAX_SPEED = 255;
  const int DEADBAND  = 0;
  const uint32_t PWM_FREQ_HZ = 2000;

  DriveTrain::DriveTrainPins driveTrainPins() {
    return DriveTrain::DriveTrainPins(
      DriveTrain::MotorPins(FL_A, FL_B, INVERT_FL),
      DriveTrain::MotorPins(FR_A, FR_B, INVERT_FR),
      DriveTrain::MotorPins(RL_A, RL_B, INVERT_RL),
      DriveTrain::MotorPins(RR_A, RR_B, INVERT_RR)
    );
  }

  // -------------------- Protocol --------------------
  const int THR_SIGN = +1;
  const int STR_SIGN = +1;

  // -------------------- Bumper Switch--------------------
  const gpio_num_t BUMPER_PIN = GPIO_NUM_15;
  const bool BUMPER_ACTIVE_LOW = false;

   // -------------------- I2C Bus --------------------
  const I2CBusConfig I2C(&Wire, 21, 22, 100000, true);


  // -------------------- TOF --------------------
  const TofConfig TOF_CFG[] = {
    { .xshutPin = 35, .addr = 0x29 }
  };


  const size_t TOF_CFG_COUNT = sizeof(TOF_CFG) / sizeof(TOF_CFG[0]);

  const bool TOF_AUTORECOVERY = true;
  const int  TOF_RECOVERY_THRESHOLD = 5;

  const int WIRE_CLOCK = 100000;
  const uint8_t I2C_SDA = 21;
  const uint8_t I2C_SCL = 22;

  // -------------------- LEDs --------------------
  const int NUM_LEDS = 32;
  const int LED_PIN  = 23;

  // -------------------- Feeder --------------------
  const FeederConfig feederConfig = {
    16,//signal pin
    10,
    45,
    50,
    500,
    2500
  };
}