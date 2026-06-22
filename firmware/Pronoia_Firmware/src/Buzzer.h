// =============================================================================
// Buzzer.h
// =============================================================================
// Non-blocking buzzer driver. Supports single beeps, beep codes, and continuous
// tone modes.
//
// PROMETHEUS HARDWARE NOTES:
//   - UI buzzer (BUZZER1): GSD9605YB-3V2800, low-side switched by Q6 (AO3400A),
//     powered by +3.3V (always on when Teensy is powered).
//   - Recovery buzzer: low-side switched by Q5 (AO3400A), powered by
//     RecoveryBuzzPower net (verify this is wired to upstream-of-FingerTech
//     battery rail on your actual board — schematic shows a deleted connector).
//   - Both gates have 1ohm series resistors and 10kohm pull-downs.
//
// USAGE:
//   1. Construct with the buzzer's GPIO pin
//   2. Call begin() once at startup
//   3. Call update() every loop iteration to service the beep state machine
// =============================================================================

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

class Buzzer {
  public:
    Buzzer(uint8_t pin, const char* name);

    void begin();
    void update();

    // Single beep, configurable duration (ms)
    void beepOnce(uint32_t durationMs = 200);

    // Beep N times with short on/off pattern, then a long pause, then repeat.
    // Set count=0 to stop the pattern.
    void beepPattern(uint8_t count, uint32_t onMs = 150, uint32_t offMs = 150,
                     uint32_t pauseMs = 1500);

    // Continuous tone — useful for recovery buzzer after touchdown
    void setContinuous(bool on);

    // Stop all beeping immediately
    void silence();

    bool isActive() const { return _mode != IDLE; }

  private:
    enum Mode { IDLE, ONCE, PATTERN, CONTINUOUS };

    uint8_t _pin;
    const char* _name;
    Mode _mode = IDLE;

    uint32_t _stateChangeMs = 0;
    bool _pinState = false;

    uint8_t _patternCount = 0;
    uint8_t _patternIndex = 0;
    uint32_t _patternOnMs = 150;
    uint32_t _patternOffMs = 150;
    uint32_t _patternPauseMs = 1500;
    bool _inPause = false;

    uint32_t _onceDurationMs = 200;

    void writePin(bool state);
};

#endif