#pragma once
#include <Arduino.h>
#include <DCMotor_L9110.h>


class DriveTrain {
    public:
        // Semantic labels so you don't memorize ordering
        enum class Wheel : uint8_t { FL, FR, RL, RR };

     // NOTE:
        // For DCMotor_L9110:
        //   dirPin -> digital direction pin (HIGH/LOW)
        //   pwmPin -> PWM pin controlling motor power (0..255)
       struct MotorPins {
            uint8_t dirPin;   // Direction control pin
            uint8_t pwmPin;   // PWM (speed) control pin
            bool invert = false; // Flip motor direction if mounted/wired mirrored
            // Default: "unset" pins (255 is a common sentinel for "not configured")
            constexpr MotorPins() : dirPin(255), pwmPin(255), invert(false) {}

            // Normal constructor
            constexpr MotorPins(uint8_t dir, uint8_t pwm, bool inv = false)
                : dirPin(dir), pwmPin(pwm), invert(inv) {}
        };

        struct DriveTrainPins {
            MotorPins fl;
            MotorPins fr;
            MotorPins rl;
            MotorPins rr;
           // Default constructor (now pins_{} works)
            constexpr DriveTrainPins() = default;

            // Convenience constructor
            constexpr DriveTrainPins(MotorPins _fl, MotorPins _fr, MotorPins _rl, MotorPins _rr)
                : fl(_fl), fr(_fr), rl(_rl), rr(_rr) {}
        };

        // --- Construction / init ---
        DriveTrain() = default;
        explicit DriveTrain(const DriveTrainPins& pins);

        // Call once in setup(). If you use the constructor config, begin() can be pinless.
        bool begin(const DriveTrainPins& pins);
        bool begin(); // uses pins provided in constructor
        void forward(int speed);
        void reverse(int speed);
        void stop();
        void turnLeft(int speed);
        void turnRight(int speed);
        // later add more complex maneuvers HERE.


        // --- Nice "driver" API  ---
        // throttle: -max..+max  (forward/back)
        // steer:    -max..+max  (left/right)
        void arcadeDrive(int throttle, int steer);



        // --- Per-wheel control todo ---
        //void setWheel(Wheel w, int speed);
        //void stopWheel(Wheel w);

        // --- Tuning / UX knobs ---
        void setMaxSpeed(int maxSpeed);     // default 255
        void setDeadband(int deadband);     // default 0..20 typical
        void setInvert(Wheel w, bool invert);

        // Debug / introspection
        DriveTrainPins getPins() const;

        void setPwmFrequency(uint32_t hz);

        //Hel
    private:

        // --------Helpers
        // clamping & deadband are used when using joystick or mixing inputs
        // Adjusts speed to be within allowed range
        static int clampSpeed(int speed, int maxSpeed);
        //Deadband is when small speeds are ignored to prevent drift
        //Ignores joystick noise and motor twitch
        int applyDeadband(int speed) const;
        //left side and right side motors
        void setLeft(int speed);
        void setRight(int speed);

        // Internal state
        DriveTrainPins pins_{};
        bool initialized_ = false;
        int maxSpeed_ = 255;
        int deadband_ = 0;

        uint32_t pwmFreqHz_ = 20000; // default PWM frequency
        //------ MOTORS -----
        //front motors
        //DCMotor_L9110 fl_;
        //DCMotor_L9110 fr_;
        //rear motors
        //DCMotor_L9110 rl_;
        //DCMotor_L9110 rr_;
};