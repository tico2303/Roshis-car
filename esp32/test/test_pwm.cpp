// Minimal L9110 Motor POC (ESP32)
// - Two-pin control per motor channel (A/B)
// - Forward:  A=PWM, B=LOW
// - Reverse:  A=LOW, B=PWM
// - Stop:     A=LOW, B=LOW
//

#include <Arduino.h>


// Pick ONE motor channel first (rear-left for example)
static uint8_t PIN_A = 27;   // "Input A" on the L9110 channel
static uint8_t PIN_B = 14;   // "Input B" on the L9110 channel

// PWM settings
static uint32_t PWM_FREQ_HZ = 2000; // try 10000, 16000, 20000, 24000
static int PWM_DUTY = 255;           // 0..255
static int PWM_DUTY_MED = 130;
static bool INVERT = false;          // flip logical direction if motor mounted mirrored

// Timing
static uint32_t RUN_MS = 1500;
static uint32_t STOP_MS = 1000;

// On ESP32, analogWrite attaches PWM hardware to the pin. We zero PWM on both pins
// before switching direction to avoid “pin still PWM-controlled” weirdness.
static bool ZERO_BOTH_BEFORE_SWITCH = true;

// ---------------------------------------------------------

static inline int clampDuty(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 255) duty = 255;
  return duty;
}

void motor_setup(uint8_t a, uint8_t b, uint32_t pwmFreqHz) {
  PIN_A = a;
  PIN_B = b;
  PWM_FREQ_HZ = pwmFreqHz;

  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);

  //init to low
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);

#if defined(ARDUINO_ARCH_ESP32)
  // Applies to analogWrite() on many ESP32 Arduino cores
  analogWriteFrequency(PWM_FREQ_HZ);
#endif

  // Also make sure PWM is "zeroed" on both pins (deterministic idle)
  analogWrite(PIN_A, 0);
  analogWrite(PIN_B, 0);
}

void motor_stop() {
  // Ensure PWM is off, then force both LOW.
  analogWrite(PIN_A, 0);
  analogWrite(PIN_B, 0);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
}

// Signed command: + = forward, - = reverse, 0 = stop
void motor_drive(int signedPower /* -255..255 */) {
  signedPower = constrain(signedPower, -255, 255);
  if (INVERT) signedPower = -signedPower;

  if (signedPower == 0) {
    motor_stop();
    return;
  }

  int duty = clampDuty(abs(signedPower));

  if (ZERO_BOTH_BEFORE_SWITCH) {
    // release PWM control from both pins before re-assigning
    analogWrite(PIN_A, 0);
    analogWrite(PIN_B, 0);
  }

  if (signedPower > 0) {
    // Forward: A=PWM, B=LOW
    digitalWrite(PIN_B, LOW);
    analogWrite(PIN_A, duty);
  } else {
    // Reverse: A=LOW, B=PWM
    digitalWrite(PIN_A, LOW);
    analogWrite(PIN_B, duty);
  }
}

// Convenience wrappers to tweak direction behavior easily
void motor_forward(int duty) { motor_drive(+clampDuty(duty)); }
void motor_reverse(int duty) { motor_drive(-clampDuty(duty)); }

// ---------------------- Arduino sketch ----------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("L9110 Motor POC starting...");

  motor_setup(PIN_A, PIN_B, PWM_FREQ_HZ);

  Serial.print("Pins A/B = ");
  Serial.print(PIN_A);
  Serial.print("/");
  Serial.println(PIN_B);

  Serial.print("PWM_FREQ_HZ = ");
  Serial.println(PWM_FREQ_HZ);

  Serial.print("PWM_DUTY = ");
  Serial.println(PWM_DUTY);

  Serial.print("INVERT = ");
  Serial.println(INVERT ? "true" : "false");

  Serial.println("Setup done.");
}

void loop() {
  Serial.println("FORWARD");
  motor_forward(PWM_DUTY);
  delay(RUN_MS);
  motor_forward(PWM_DUTY_MED);
  delay(RUN_MS);

  Serial.println("STOP");
  motor_stop();
  delay(STOP_MS);

  Serial.println("REVERSE");
  motor_reverse(PWM_DUTY);
  delay(RUN_MS);

  Serial.println("STOP");
  motor_stop();
  delay(2000);
}