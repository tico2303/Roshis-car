#include <Arduino.h>
#include <ESP32Encoder.h>

// ----------------- MOTOR DRIVER (DRV8871 EN/PH) -----------------
static const int PIN_DRV_EN = 27;   // PWM
static const int PIN_DRV_PH = 14;   // Direction

// ----------------- ENCODER PINS (AFTER VOLTAGE DIVIDER) ---------
static const int PIN_ENC_A = 19;
static const int PIN_ENC_B = 18;

// ----------------- PWM SETTINGS --------------------------------
static const int PWM_CH   = 0;
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;      // 0..255

// ----------------- ENCODER / WHEEL CONFIG (OPTIONAL) -----------
static const int   SAMPLE_MS = 100;     // telemetry period
static const float ENCODER_PPR = 11.0f; // pulses per motor-shaft rev (adjust)
static const float GEAR_RATIO  = 30.0f; // motor revs per wheel rev (adjust)
static const float QUAD_FACTOR = 2.0f;  // HalfQuad typically 2x on many setups (library-dependent)
static const float PULSES_PER_WHEEL_REV = ENCODER_PPR * GEAR_RATIO * QUAD_FACTOR;

// ----------------------------------------------------------------
ESP32Encoder enc;

static void setMotor(int dir, uint8_t pwm) {
  if (dir == 0 || pwm == 0) {
    ledcWrite(PWM_CH, 0);
    return;
  }
  digitalWrite(PIN_DRV_PH, (dir > 0) ? HIGH : LOW);
  ledcWrite(PWM_CH, pwm);
}

static void runPhase(const char* name, int dir, uint8_t pwm, uint32_t duration_ms) {
  Serial.printf("\n=== %s (dir=%d pwm=%u) ===\n", name, dir, pwm);

  // Reset encoder for this phase
  enc.clearCount();
  int64_t last = 0;

  setMotor(dir, pwm);

  const uint32_t start = millis();
  uint32_t lastPrint = start;

  while (millis() - start < duration_ms) {
    const uint32_t now = millis();

    if (now - lastPrint >= SAMPLE_MS) {
      lastPrint = now;

      const int a = digitalRead(PIN_ENC_A);
      const int b = digitalRead(PIN_ENC_B);

      const int64_t total = enc.getCount();
      const int64_t dticks = total - last;
      last = total;

      // ticks/sec
      const float dt_sec = SAMPLE_MS / 1000.0f;
      const float ticks_per_sec = (float)dticks / dt_sec;

      // wheel rad/s (optional)
      float wheel_rad_s = 0.0f;
      if (PULSES_PER_WHEEL_REV > 0.0f) {
        const float wheel_rev_s = ticks_per_sec / PULSES_PER_WHEEL_REV;
        wheel_rad_s = wheel_rev_s * 2.0f * 3.1415926535f;
      }

      Serial.printf("A=%d B=%d  dticks=%lld  total=%lld  wheel_rad_s=%.2f\n",
                    a, b,
                    (long long)dticks,
                    (long long)total,
                    wheel_rad_s);
    }
    delay(1);
  }

  setMotor(0, 0);
  Serial.printf("=== %s done. final ticks=%lld ===\n", name, (long long)enc.getCount());
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor driver
  pinMode(PIN_DRV_PH, OUTPUT);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_DRV_EN, PWM_CH);
  ledcWrite(PWM_CH, 0);

  // Encoder pins: with voltage divider, prefer no internal pulls
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  // If your encoder outputs are open-collector AND you have no external pullups,
  // you might need internal pull-ups (but with dividers, usually NOT).
  ESP32Encoder::useInternalWeakPullResistors = puType::none;

  // Attach encoder
  enc.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
  enc.clearCount();

  Serial.println("\nMotor + ESP32Encoder integration test starting...");
}

void loop() {
  runPhase("FORWARD", +1, 200, 3000);
  delay(800);

  runPhase("STOP", 0, 0, 1000);
  delay(800);

  runPhase("REVERSE", -1, 140, 3000);
  delay(800);

  runPhase("STOP", 0, 0, 1000);
  delay(1500);
}