// =============================================================================
// PyroChannel.cpp
// =============================================================================
#include "PyroChannel.h"

PyroChannel::PyroChannel(uint8_t firePin, uint8_t contPin, const char* name)
  : _firePin(firePin), _contPin(contPin), _name(name) {
}

void PyroChannel::begin() {
  // CRITICAL: Set fire pin LOW BEFORE pinMode() configures it as OUTPUT.
  // This minimizes the window where the pin might glitch HIGH during boot.
  // The hardware 10kohm gate pull-down (R4) provides defense in depth.
  digitalWrite(_firePin, LOW);
  pinMode(_firePin, OUTPUT);
  digitalWrite(_firePin, LOW);  // Belt-and-suspenders

  // Continuity sense pin: the voltage divider provides a ~3.0V signal when
  // active. No internal pull is needed because the divider's 10kohm leg
  // provides a defined LOW state when the divider is unpowered.
  pinMode(_contPin, INPUT);

  _armed = false;
  _firing = false;
}

bool PyroChannel::hasContinuity() const {
  // ACTIVE-HIGH per Prometheus circuit:
  // 8.4V_ARMED -- 18kohm -- [sense pin] -- 10kohm -- GND
  // Pin reads ~3.0V (HIGH) when ematch is present AND FingerTech is ON.
  return digitalRead(_contPin) == HIGH;
}

bool PyroChannel::fire(uint32_t durationMs) {
  // ----- SAFETY INTERLOCK CHAIN -----
  if (!_armed) {
    return false;
  }
  if (_firing) {
    return false;
  }
  if (!hasContinuity()) {
    // No load detected. Could mean:
    //   (a) ematch is open / not connected
    //   (b) FingerTech is OFF
    // Either way, firing is refused.
    return false;
  }
  if (durationMs == 0 || durationMs > 1000) {
    // Sanity-check duration: must be > 0 and <= 1 second
    return false;
  }
  // ----- ALL CHECKS PASSED — INITIATE FIRE -----
  digitalWrite(_firePin, HIGH);
  _firing = true;
  _fireStartMs = millis();
  _fireDurationMs = durationMs;
  return true;
}

void PyroChannel::update() {
  if (_firing) {
    if (millis() - _fireStartMs >= _fireDurationMs) {
      digitalWrite(_firePin, LOW);
      _firing = false;
    }
  }
}

void PyroChannel::forceOff() {
  digitalWrite(_firePin, LOW);
  _firing = false;
  _armed = false;
}