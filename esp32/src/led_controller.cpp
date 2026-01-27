#include "led_controller.h"
#include <FastLED.h>

LedController::LedController(int pin, int numLeds)
    : _pin(pin), _numLeds(numLeds), _mode(OFF)
{
    _leds = new CRGB[_numLeds];
}

void LedController::begin() {
    const int PIN = 23; // keep hardcoded for now
    FastLED.addLeds<WS2812B, PIN, GRB>(_leds, _numLeds);
    FastLED.setBrightness(80);

    _dirty = true;
    _mode = OFF;
    update(); // force initial clear
}

// ================== PUBLIC MODES ==================

void LedController::angry() {
    _mode = ANGRY;
    _dirty = true;
    _lastUpdate = 0;
    _lastStepMs = 0;
}

void LedController::blink() {
    _mode = BLINK_EYE;
    _dirty = true;
    _blinkState = false;
    _lastUpdate = 0;
}

void LedController::idle() {
    _mode = SOLID;
    _dirty = true;
    _solidColor = CRGB::Blue;
}

void LedController::thinking() {
    _mode = SOLID;
    _dirty = true;
    _solidColor = CRGB::Aquamarine;
}

void LedController::sleepy() {
    _mode = PULSING;
    _dirty = true;
    _pulseColor = CRGB::Blue;
    _pulseBpm = 12;
    _pulseLowB = 5;
    _pulseHighB = 80;
}

void LedController::rainbow() {
    _mode = RAINBOW;
    _dirty = true;
    _lastStepMs = 0;
}

void LedController::off() {
    _mode = OFF;
    _dirty = true;
}
void LedController::setEyeConfig(int ringLen, int topIndex) {
    _eyeRingLen = (ringLen <= 0) ? 16 : ringLen;
    _eyeTopIndex = topIndex;
    _dirty = true;
}

void LedController::setEyeColors(CRGB eyeWhite, CRGB iris) {
    _eyeColor = eyeWhite;
    _irisColor = iris;
    _dirty = true;
}

void LedController::lookAt(uint8_t irisPos) {
    _mode = EYES;
    _dirty = false;
    _irisPos = (_eyeRingLen > 0) ? (irisPos % _eyeRingLen) : irisPos;

}

void LedController::debugTopIndex(int ringLen) {
  static int i = 0;
  fill_solid(_leds, _numLeds, CRGB::Black);

  // light the same index on both rings if you have 2x16
  int rings = _numLeds / ringLen;
  for (int r = 0; r < rings; r++) {
    _leds[r * ringLen + i] = CRGB::Red;
  }

  FastLED.show();

  Serial.print("index = ");
  Serial.println(i);

  i = (i + 1) % ringLen;
  delay(200);
}
void LedController::debugShowIndex(int ringLen, uint8_t idx) {
  fill_solid(_leds, _numLeds, CRGB::Black);

  int rings = _numLeds / ringLen;
  idx = idx % ringLen;

  for (int r = 0; r < rings; r++) {
    _leds[r * ringLen + idx] = CRGB::Red;
  }

  FastLED.show();
}

// ================== UPDATE LOOP ==================

void LedController::update() {
    const unsigned long now = millis();

    switch (_mode) {

        case ANGRY: {
            // Recipe option A: fast red pulse
            _renderPulse(CRGB::Red, 60, 20, 200);

            // Recipe option B: angry flicker
            // _renderFlicker(CRGB::Red, 80, 30, now);
            break;
        }

        case SOLID: {
            /*
            if (_dirty) {
                _dirty = false;
                FastLED.setBrightness(80);
                _renderSolid(_solidColor);
            }
            break;
            */
            static uint8_t pos = 0;
            static unsigned long last = 0;
            if (millis() - last > 120) {
                last = millis();
                pos = (pos + 1) % 16;
            }
            _renderEyesIris(16, 0, pos, CRGB::Black, CRGB::Green, 1, 200);
            break;
        }

        case PULSING: {
            // uses configured pulse params from sleepy() (or other future modes)
            _renderPulse(_pulseColor, _pulseBpm, _pulseLowB, _pulseHighB);
            break;
        }

        case RAINBOW: {
            if (_throttleEvery(20, now)) {
                FastLED.setBrightness(80);
                fill_rainbow(_leds, _numLeds, _hue++, 7);
                FastLED.show();
            }
            break;
        }
        case BLINK_EYE: {
            // ringLen=16
            _renderBlinkEye(
                /*ringLen=*/16,
                /*topIndex=*/14,
                /*openColor=*/CRGB::Amethyst,
                /*closedColor=*/CRGB::Black,
                /*closeMs=*/120,
                /*holdClosedMs=*/80,
                /*openMs=*/140,
                /*holdOpenMs=*/900,
                now
            );
            break;
        }
        case EYES: {
            _renderEyesIris(
                _eyeRingLen,
                _eyeTopIndex,
                _irisPos,
                _eyeColor,
                _irisColor,
                _irisSize,
                _eyeDim
            );
            break;
        }

        case OFF:
        default: {
            if (_dirty) {
                _dirty = false;
                FastLED.setBrightness(0);
                _renderSolid(CRGB::Black);
                FastLED.setBrightness(80); // restore baseline (optional)
            }
            break;
        }
    }
}

// ================== Helpers ==================

// limits how often a mode updates
bool LedController::_throttleEvery(uint16_t ms, unsigned long now) {
    if (now - _lastStepMs < ms) return false;
    _lastStepMs = now;
    return true;
}

void LedController::_renderSolid(CRGB c) {
    fill_solid(_leds, _numLeds, c);
    FastLED.show();
}

// breathing/pulse
void LedController::_renderPulse(CRGB c, uint16_t bpm, uint8_t lowB, uint8_t highB) {
    if (bpm < 1) bpm = 1;
    uint8_t b = beatsin8(bpm, lowB, highB);
    FastLED.setBrightness(b);
    fill_solid(_leds, _numLeds, c);
    FastLED.show();
}

void LedController::_renderBlink(CRGB openC, CRGB closedC, uint16_t onMs, uint16_t offMs, unsigned long now) {
    uint16_t interval = _blinkState ? onMs : offMs;
    if (now - _lastUpdate >= interval) {
        _lastUpdate = now;
        _blinkState = !_blinkState;
    }

    FastLED.setBrightness(80);
    fill_solid(_leds, _numLeds, _blinkState ? openC : closedC);
    FastLED.show();
}

void LedController::_renderFlicker(CRGB base, uint8_t intensity, uint16_t updateMs, unsigned long now) {
    if (!_throttleEvery(updateMs, now)) return;

    uint8_t jitter = random8(intensity);
    uint8_t b = qsub8(180, jitter);
    FastLED.setBrightness(b);

    CHSV hsv = rgb2hsv_approximate(base);
    hsv.hue += random8(intensity / 2);
    hsv.val = 255;

    fill_solid(_leds, _numLeds, hsv);
    FastLED.show();
}

void LedController::_renderSparkle(CRGB base, CRGB spark, uint8_t density, uint16_t updateMs, unsigned long now) {
    // base layer (once)
    if (_dirty) {
        _dirty = false;
        fill_solid(_leds, _numLeds, base);
    }

    if (!_throttleEvery(updateMs, now)) return;

    fadeToBlackBy(_leds, _numLeds, 20);

    if (random8() < density) {
        int i = random16(_numLeds);
        _leds[i] = spark;
    }

    FastLED.setBrightness(80);
    FastLED.show();
}
static inline int wrapIndex(int i, int n) {
    i %= n;
    if (i < 0) i += n;
    return i;
}

void LedController::_renderBlinkEye(
    int ringLen,
    int topIndex,
    CRGB openColor,
    CRGB closedColor,
    uint16_t closeMs,
    uint16_t holdClosedMs,
    uint16_t openMs,
    uint16_t holdOpenMs,
    unsigned long now
) {
    // Safety
    if (ringLen <= 0 || _numLeds <= 0) return;
    if (ringLen > _numLeds) ringLen = _numLeds;

    // Determine how many rings are chained
    const int rings = _numLeds / ringLen;
    const int layersMax = ringLen / 2; // enough layers to cover the ring

    // Step interval for closing/opening (at least 10ms so we don't go nuts)
    const uint16_t closeStepMs = (uint16_t)max<uint32_t>(10, (uint32_t)closeMs / max(1, layersMax));
    const uint16_t openStepMs  = (uint16_t)max<uint32_t>(10, (uint32_t)openMs  / max(1, layersMax));

    // Advance blink state machine on schedule
    if (now >= _eyeNextMs) {
        switch (_eyePhase) {
            case EYE_OPEN_HOLD:
                _eyePhase = EYE_CLOSING;
                _eyeLayer = 0;
                _eyeNextMs = now; // advance immediately
                break;

            case EYE_CLOSING:
                if (_eyeLayer < layersMax) {
                    _eyeLayer++;
                    _eyeNextMs = now + closeStepMs;
                } else {
                    _eyePhase = EYE_CLOSED_HOLD;
                    _eyeNextMs = now + holdClosedMs;
                }
                break;

            case EYE_CLOSED_HOLD:
                _eyePhase = EYE_OPENING;
                _eyeNextMs = now; // advance immediately
                break;

            case EYE_OPENING:
                if (_eyeLayer > 0) {
                    _eyeLayer--;
                    _eyeNextMs = now + openStepMs;
                } else {
                    _eyePhase = EYE_OPEN_HOLD;
                    _eyeNextMs = now + holdOpenMs;
                }
                break;
        }
    }

    // Render: start fully open, then close "layers" symmetrically from the top
    // We render each ring segment independently.
    FastLED.setBrightness(80);

    for (int r = 0; r < rings; r++) {
        const int base = r * ringLen;

        // 1) open frame
        for (int i = 0; i < ringLen; i++) {
            _leds[base + i] = openColor;
        }

        // 2) apply closing layers: layer 1 closes top; layer 2 closes +/-1; etc.
        // If _eyeLayer == 0 -> fully open
        for (int d = 0; d < _eyeLayer; d++) {
            int i1 = wrapIndex(topIndex + d, ringLen);
            int i2 = wrapIndex(topIndex - d, ringLen);
            _leds[base + i1] = closedColor;
            _leds[base + i2] = closedColor;
        }
    }

    FastLED.show();
}

void LedController::_renderEyesIris(
    int ringLen,
    int topIndex,
    uint8_t irisPos,
    CRGB eyeColor,
    CRGB irisColor,
    uint8_t irisSize,
    uint8_t eyeDim
) {
    if (ringLen <= 0 || _numLeds <= 0) return;
    if (_numLeds < ringLen) return;

    const int rings = _numLeds / ringLen;
    if (rings <= 0) return;

    irisPos %= ringLen;
    topIndex = wrapIndex(topIndex, ringLen);
    if (irisSize < 1) irisSize = 1;

    // Convert irisPos relative to topIndex into absolute ring index
    int irisIndex = wrapIndex(topIndex + irisPos, ringLen);

    // Dim base eye color
    CRGB base = eyeColor;
    base.nscale8_video(eyeDim);

    FastLED.setBrightness(80);

    for (int r = 0; r < rings; r++) {
        int baseIdx = r * ringLen;

        // Fill eye base
        for (int i = 0; i < ringLen; i++) {
            _leds[baseIdx + i] = base;
        }

        // Draw iris (single LED or blob)
        int half = irisSize / 2;
        for (int d = -half; d <= half; d++) {
            int idx = wrapIndex(irisIndex + d, ringLen);
            _leds[baseIdx + idx] = irisColor;
        }
    }

    FastLED.show();
}