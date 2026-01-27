#pragma once
#include <FastLED.h>

class LedController {
public:
    enum Mode {
        OFF,
        SOLID,
        BLINK_EYE,
        ANGRY,
        RAINBOW,
        PULSING,
        EYES,
    };

    LedController(int pin, int numLeds);

    void begin();
    void update(); // call this in loop()

    // Semantic behaviors (set mode + parameters; update() renders)
    void angry();
    void blink();
    void idle();
    void thinking();
    void sleepy();
    void rainbow();
    void off();
     // Eyes / iris control
    void lookAt(uint8_t irisPos);                 // 0..15 relative to topIndex
    void setEyeConfig(int ringLen, int topIndex); // optional: configure ring geometry
    void setEyeColors(CRGB eyeWhite, CRGB iris);  // optional


    void debugTopIndex(int ringlen);
    void debugShowIndex(int ringLen, uint8_t idx);

private:
    int _pin = 0;
    int _numLeds = 0;
    Mode _mode = OFF;

    CRGB* _leds = nullptr;

    // One-shot render guard for SOLID/OFF/etc.
    bool _dirty = true;

    // ---- Mode-configurable parameters (set by semantic methods) ----
    CRGB _solidColor = CRGB::Blue;

    CRGB _pulseColor = CRGB::Aquamarine;
    uint16_t _pulseBpm = 24;     // beats per minute
    uint8_t _pulseLowB = 10;
    uint8_t _pulseHighB = 180;

    // ---- Animation / timing state ----
    unsigned long _lastUpdate = 0; // e.g., blink toggles
    unsigned long _lastStepMs = 0; // throttle helper
    bool _blinkState = false;
    uint8_t _hue = 0;

    // Eye blink state machine (non-blocking)
    enum EyeBlinkPhase { EYE_OPEN_HOLD, EYE_CLOSING, EYE_CLOSED_HOLD, EYE_OPENING };

    EyeBlinkPhase _eyePhase = EYE_OPEN_HOLD;
    uint8_t _eyeLayer = 0;                 // how many "layers" are closed (0..ringLen/2)
    unsigned long _eyeNextMs = 0;          // next time to advance the blink

    // Eye config/state
    int _eyeRingLen = 16;      // LEDs per ring
    int _eyeTopIndex = 0;      // physical top LED index within a ring
    uint8_t _irisPos = 0;      // 0.._eyeRingLen-1 relative to top
    uint8_t _irisSize = 1;     // 1 = single LED iris
    uint8_t _eyeDim = 180;     // dim the "white" so iris pops
    CRGB _eyeColor = CRGB::White;
    CRGB _irisColor = CRGB::Blue;

    // ---- Helpers (colors are always passed in) ----
    bool _throttleEvery(uint16_t ms, unsigned long now);

    void _renderSolid(CRGB c);
    void _renderPulse(CRGB c, uint16_t bpm, uint8_t lowB, uint8_t highB);
    void _renderBlink(CRGB openC, CRGB closedC, uint16_t onMs, uint16_t offMs, unsigned long now);
    void _renderFlicker(CRGB base, uint8_t intensity, uint16_t updateMs, unsigned long now);
    void _renderSparkle(CRGB base, CRGB spark, uint8_t density /*0-255*/, uint16_t updateMs, unsigned long now);
    // New primitive
    void _renderBlinkEye(
        int ringLen,
        int topIndex,
        CRGB openColor,
        CRGB closedColor,
        uint16_t closeMs,
        uint16_t holdClosedMs,
        uint16_t openMs,
        uint16_t holdOpenMs,
        unsigned long now
    );
    void _renderEyesIris(
        int ringLen,
        int topIndex,          // physical top LED index
        uint8_t irisPos,        // 0..ringLen-1 relative to top
        CRGB eyeColor,
        CRGB irisColor,
        uint8_t irisSize = 1,
        uint8_t eyeDim = 255
    );
};