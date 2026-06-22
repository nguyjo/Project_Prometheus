// =============================================================================
// PyroChannel.h
// =============================================================================
// Single pyro channel abstraction with built-in safety interlocks.
//
// CIRCUIT-SPECIFIC NOTES FOR PROMETHEUS:
//   - Fire MOSFET (AO3400A) is low-side switched, gate driven by Teensy fire pin.
//     R3 (100ohm) is gate series resistor, R4 (10kohm) is gate pull-down to GND.
//   - Continuity sense is a voltage divider from +8.4V_ARMED (downstream of
//     FingerTech) through 18kohm down to sense pin, then 10kohm to GND.
//     Sense pin reads ~3.0V (HIGH) when ematch present AND FingerTech ON.
//     Sense pin reads ~0V (LOW) when EITHER ematch absent OR FingerTech OFF.
//   - Therefore: continuity sensing is only meaningful when FingerTech is ON.
//     The firmware cannot distinguish "ematch missing" from "FingerTech off"
//     using continuity sense alone.
//
// USAGE:
//   1. Construct one PyroChannel per physical pyro output
//   2. Call begin() once at startup — sets fire pin LOW immediately
//   3. Call update() every loop iteration — services the fire-pulse timer
//   4. Call hasContinuity() to read the divider output
//   5. Call fire() to initiate a pulse (multiple safety conditions must be met)
//
// SAFETY INTERLOCKS (all must be true for fire() to actually fire):
//   - Software armed flag is set
//   - Continuity is present on the channel
//   - Channel is not already firing
//   - Bounded fire duration (max 1000 ms)
//
// The FingerTech hardware power switch gates the high-current path AND the
// continuity divider — it represents the final independent layer of safety.
// =============================================================================

#ifndef PYROCHANNEL_H
#define PYROCHANNEL_H

#include <Arduino.h>

class PyroChannel {
  public:
    PyroChannel(uint8_t firePin, uint8_t contPin, const char* name);

    // Setup — MUST be called before any other method.
    // Forces fire pin LOW immediately to ensure safe boot state.
    void begin();

    // Call every loop iteration — services the fire-pulse timer.
    void update();

    // Read continuity sense pin.
    // Returns true if continuity divider is energized AND load is connected.
    // ACTIVE-HIGH: pin reads HIGH (~3.0V) when ematch present AND FingerTech ON.
    // Reads LOW if EITHER condition fails — firmware cannot distinguish these.
    bool hasContinuity() const;

    // Attempt to fire this channel for the specified duration (ms).
    // Returns true if firing was initiated, false if any safety check failed.
    bool fire(uint32_t durationMs);

    // Force-disarm and ensure pin is LOW. Called by emergency stop logic.
    void forceOff();

    // Software arming control
    void setArmed(bool armed) { _armed = armed; }
    bool isArmed() const { return _armed; }

    // Diagnostics
    bool isFiring() const { return _firing; }
    const char* name() const { return _name; }

  private:
    uint8_t _firePin;
    uint8_t _contPin;
    const char* _name;

    bool _armed = false;
    bool _firing = false;
    uint32_t _fireStartMs = 0;
    uint32_t _fireDurationMs = 0;
};

#endif