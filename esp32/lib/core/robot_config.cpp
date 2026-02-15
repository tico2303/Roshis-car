#include "robot_config.h"
#include "config.h"

namespace RobotConfig {

  const int SERIAL_BUAD_RATE = 115200;

  // -------------------- Drivetrain --------------------
  /*
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
  */
const MotorPins LEFT_PINS = {
  .in1 = 25,
  .in2 = 26,
  .encA = 32,
  .encB = 33
};

const MotorPwmConfig LEFT_PWM = {
  .ch_in1 = 0,
  .ch_in2 = 1,
  .freq_hz = 20000,
  .res_bits = 8
};

const MotorPins RIGHT_PINS ={
  .in1 = 27,
  .in2 = 14,
  .encA = 19,
  .encB = 18
};

const MotorPwmConfig RIGHT_PWM = {
  .ch_in1 = 2,
  .ch_in2 = 3,
  .freq_hz = 20000,
  .res_bits = 8
};

const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_LEFT_ENCODER = false;
const bool INVERT_RIGHT_MOTOR = true;
const bool INVERT_RIGHT_ENCODER = true;

  // -------------------- DriveBase tuning --------------------
  // 11 PPR encoder * 30:1 gear ratio * 2 (half-quad) = 660
  const float PULSES_PER_WHEEL_REV = 660.0f;
  const uint32_t CONTROL_PERIOD_MS = 20;

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

// ----- LEDC PWM -----
inline constexpr uint32_t PWM_FREQ_HZ = 20000;
inline constexpr uint8_t  PWM_BITS    = 10;
inline constexpr uint16_t PWM_MAX     = (1u << PWM_BITS) - 1;

inline constexpr int LEFT_PWM_CH1  = 0;
inline constexpr int LEFT_PWM_CH2  = 1;
inline constexpr int RIGHT_PWM_CH1 = 2;
inline constexpr int RIGHT_PWM_CH2 = 3;

// ----- Encoders  pulse counts-----
inline constexpr int LEFT_PCNT_UNIT  = 0;
inline constexpr int RIGHT_PCNT_UNIT = 1;

// Encoder model params (tune these)
inline constexpr int32_t ENCODER_PPR = 11;     // pulses per motor shaft rev
inline constexpr float   GEAR_RATIO  = 30.0f;  // motor revs per wheel rev

inline constexpr int32_t QUAD_FACTOR = 2;

inline constexpr float PULSES_PER_WHEEL_REV =
  (float)ENCODER_PPR * GEAR_RATIO * (float)QUAD_FACTOR;

// ----- Robot mapping -----
inline constexpr float MAX_NORM_THR = 1000.0f; // if thr is -1000..1000 (adjust to your Pi)
inline constexpr float MAX_NORM_STR = 1000.0f;

inline constexpr uint32_t CONTROL_PERIOD_MS  = 20;
inline constexpr uint32_t COMMAND_TIMEOUT_MS = 250;
