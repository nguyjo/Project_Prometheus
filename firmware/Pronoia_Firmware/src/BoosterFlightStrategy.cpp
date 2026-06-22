// =============================================================================
// BoosterFlightStrategy.cpp
// =============================================================================
#include "BoosterFlightStrategy.h"
#include "StateEstimator.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "PyroChannel.h"
#include "Buzzer.h"
#include "Config.h"
#include <math.h>

BoosterFlightStrategy::BoosterFlightStrategy(
    StateEstimator& estimator,
    AccelerationSource& accelSrc,
    Barometer& baro,
    PyroChannel& mainPyro,
    Buzzer& uiBuzzer,
    Buzzer& recoveryBuzzer)
  : _estimator(estimator),
    _accelSrc(accelSrc),
    _baro(baro),
    _mainPyro(mainPyro),
    _uiBuzzer(uiBuzzer),
    _recoveryBuzzer(recoveryBuzzer) {
}

const char* BoosterFlightStrategy::currentStateName() const {
  switch (_state) {
    case PRE_LIFTOFF:    return "PRE_LIFTOFF";
    case BOOST:          return "BOOST";
    case COAST:          return "COAST";
    case DESCENT_DROGUE: return "DESCENT_DROGUE";
    case MAIN_FIRED:     return "MAIN_FIRED";
    case DESCENT_MAIN:   return "DESCENT_MAIN";
    case TOUCHDOWN:      return "TOUCHDOWN";
  }
  return "UNKNOWN";
}

bool BoosterFlightStrategy::isFlightComplete() const {
  return _state == TOUCHDOWN;
}

void BoosterFlightStrategy::update() {
  switch (_state) {
    case PRE_LIFTOFF:    updatePreLiftoff();    break;
    case BOOST:          updateBoost();         break;
    case COAST:          updateCoast();         break;
    case DESCENT_DROGUE: updateDescentDrogue(); break;
    case MAIN_FIRED:     updateMainFired();     break;
    case DESCENT_MAIN:   updateDescentMain();   break;
    case TOUCHDOWN:      updateTouchdown();     break;
  }
}

// -----------------------------------------------------------------------------
// PRE_LIFTOFF — wait for sustained high accel = booster ignition
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updatePreLiftoff() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (accel > Config::Safety::LIFTOFF_ACCEL_MPS2) {
    if (_liftoffCandidateMs == 0) {
      _liftoffCandidateMs = now;
    }
    if ((now - _liftoffCandidateMs) >= Config::Safety::LIFTOFF_DURATION_MS) {
      enterBoost();
    }
  } else {
    _liftoffCandidateMs = 0;
  }
}

void BoosterFlightStrategy::enterBoost() {
  _liftoffMs = millis();
  _state = BOOST;
  _burnoutCandidateMs = 0;
  _maxAltitudeSeen = 0.0f;

  // Reset the state estimator now that we're actually moving. Eliminates
  // any drift accumulated while sitting on the pad.
  _estimator.reset();

  _uiBuzzer.beepOnce(100);
}

// -----------------------------------------------------------------------------
// BOOST — wait for booster burnout
// -----------------------------------------------------------------------------
// Booster burns for ~3s. We detect burnout by accel dropping below threshold
// for sustained duration. After burnout, the rocket coasts upward under
// gravity alone (and the sustainer is still attached, deadweight).
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updateBoost() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (fabsf(accel) < Config::Safety::BURNOUT_ACCEL_MPS2) {
    if (_burnoutCandidateMs == 0) {
      _burnoutCandidateMs = now;
    }
    if ((now - _burnoutCandidateMs) >= Config::Safety::BURNOUT_DURATION_MS) {
      enterCoast();
    }
  } else {
    _burnoutCandidateMs = 0;
  }
}

void BoosterFlightStrategy::enterCoast() {
  _burnoutMs = millis();
  _state = COAST;
  _uiBuzzer.beepOnce(100);
}

// -----------------------------------------------------------------------------
// COAST — waiting for sustainer hot-staging + mechanical drogue release
// -----------------------------------------------------------------------------
// During COAST, the sustainer ignites and separates. The booster's drogue
// is released mechanically when the separation force triggers the release
// mechanism (~90 m/s). The booster reaches apogee shortly after, then
// begins descending under drogue.
//
// Our firmware doesn't fire anything during COAST. We just need to detect
// when we're definitely descending so we can transition to DESCENT_DROGUE.
//
// Detection: velocity is sustained negative AND we're well below the peak
// altitude we've seen. The "well below peak" check protects against velocity
// noise during the brief moments around apogee.
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updateCoast() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();

  // Track max altitude during ascent
  if (altitude > _maxAltitudeSeen) {
    _maxAltitudeSeen = altitude;
  }

  // We're definitely descending if velocity is negative AND we've fallen
  // at least 10m below our peak altitude. The "10m below peak" filter
  // avoids transient sign flips on velocity right around apogee.
  bool descending = (velocity < 0.0f) && (altitude < _maxAltitudeSeen - 10.0f);

  if (descending) {
    enterDescentDrogue();
  }
}

void BoosterFlightStrategy::enterDescentDrogue() {
  _state = DESCENT_DROGUE;
  // Two-beep pattern: drogue descent
  _uiBuzzer.beepPattern(2, 80, 80, 0);
}

// -----------------------------------------------------------------------------
// DESCENT_DROGUE — falling under booster drogue, waiting to reach 1,500 ft
// -----------------------------------------------------------------------------
// When altitude descends through 1,500 ft AGL, fire the main parachute.
// We check altitude every loop; once we're below the threshold, fire.
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updateDescentDrogue() {
  float altitude = _estimator.altitude_m();

  if (altitude < Config::Safety::MAIN_DEPLOY_ALT_M) {
    enterMainFired();
  }
}

void BoosterFlightStrategy::enterMainFired() {
  _mainFiredMs = millis();
  _state = MAIN_FIRED;

  // Force-arm and fire the main pyro. We bypass the CLI two-step arming
  // protocol because this is the autonomous flight path. The hardware
  // FingerTech switch is the final independent layer of safety — the
  // operator armed it at the pad and the umbilical has been disconnected.
  _mainPyro.setArmed(true);
  _mainPyro.fire(Config::Pyro::FIRE_TIME_MS);

  // Three-beep pattern: main deploy event
  _uiBuzzer.beepPattern(3, 50, 80, 0);
}

// -----------------------------------------------------------------------------
// MAIN_FIRED — pyro pulse in progress, wait for completion
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updateMainFired() {
  if (!_mainPyro.isFiring()) {
    enterDescentMain();
  }
}

void BoosterFlightStrategy::enterDescentMain() {
  _state = DESCENT_MAIN;
  _touchdownCandidateMs = 0;
}

// -----------------------------------------------------------------------------
// DESCENT_MAIN — falling under main, watching for touchdown
// -----------------------------------------------------------------------------
void BoosterFlightStrategy::updateDescentMain() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();
  uint32_t now = millis();

  bool slowEnough = (fabsf(velocity) < Config::Safety::TOUCHDOWN_VEL_MAX_MPS);
  bool lowEnough  = (altitude < Config::Safety::TOUCHDOWN_ALT_MAX_M);

  if (slowEnough && lowEnough) {
    if (_touchdownCandidateMs == 0) {
      _touchdownCandidateMs = now;
    }
    if ((now - _touchdownCandidateMs) >= Config::Safety::TOUCHDOWN_DURATION_MS) {
      enterTouchdown();
    }
  } else {
    _touchdownCandidateMs = 0;
  }
}

void BoosterFlightStrategy::enterTouchdown() {
  _touchdownMs = millis();
  _state = TOUCHDOWN;
  _recoveryBuzzer.setContinuous(true);
}

void BoosterFlightStrategy::updateTouchdown() {
  // Terminal state — recovery buzzer is running. Nothing to do.
}

uint8_t BoosterFlightStrategy::currentStateId() const {
  return (uint8_t)_state;  // The enum values are 0-6
}