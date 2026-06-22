// =============================================================================
// SustainerFlightStrategy.cpp
// =============================================================================
#include "SustainerFlightStrategy.h"
#include "StateEstimator.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "IMU.h"
#include "TiltEstimator.h"
#include "PyroChannel.h"
#include "Buzzer.h"
#include "RateDampingController.h"
#include "Config.h"
#include <math.h>

SustainerFlightStrategy::SustainerFlightStrategy(
    StateEstimator& estimator,
    AccelerationSource& accelSrc,
    Barometer& baro,
    IMU& imu,
    TiltEstimator& tiltEst,
    PyroChannel& ignitorPyro,
    PyroChannel& mainPyro,
    Buzzer& uiBuzzer,
    Buzzer& recoveryBuzzer,
    RateDampingController& rateDamper)
  : _estimator(estimator),
    _accelSrc(accelSrc),
    _baro(baro),
    _imu(imu),
    _tiltEst(tiltEst),
    _ignitorPyro(ignitorPyro),
    _mainPyro(mainPyro),
    _uiBuzzer(uiBuzzer),
    _recoveryBuzzer(recoveryBuzzer),
    _rateDamper(rateDamper) {
}

const char* SustainerFlightStrategy::currentStateName() const {
  switch (_state) {
    case PRE_LIFTOFF:           return "PRE_LIFTOFF";
    case STAGE1_BOOST:          return "STAGE1_BOOST";
    case SEPARATION_COAST:      return "SEPARATION_COAST";
    case TILT_CHECK:            return "TILT_CHECK";
    case SUSTAINER_IGNITION:    return "SUSTAINER_IGNITION";
    case STAGE2_BOOST:          return "STAGE2_BOOST";
    case COAST:                 return "COAST";
    case POST_APOGEE_DESCENT:   return "POST_APOGEE_DESCENT";
    case MAIN_FIRED:            return "MAIN_FIRED";
    case DESCENT_UNDER_MAIN:    return "DESCENT_UNDER_MAIN";
    case TOUCHDOWN:             return "TOUCHDOWN";
    case ABORT_DESCENT_DROGUE:  return "ABORT_DESCENT_DROGUE";
    case ABORT_MAIN_FIRED:      return "ABORT_MAIN_FIRED";
    case ABORT_DESCENT_MAIN:    return "ABORT_DESCENT_MAIN";
    case ABORT_TOUCHDOWN:       return "ABORT_TOUCHDOWN";
  }
  return "UNKNOWN";
}

uint8_t SustainerFlightStrategy::currentStateId() const {
  return (uint8_t)_state;
}

bool SustainerFlightStrategy::isFlightComplete() const {
  return _state == TOUCHDOWN || _state == ABORT_TOUCHDOWN;
}

void SustainerFlightStrategy::update() {
  switch (_state) {
    case PRE_LIFTOFF:           updatePreLiftoff();           break;
    case STAGE1_BOOST:          updateStage1Boost();          break;
    case SEPARATION_COAST:      updateSeparationCoast();      break;
    case TILT_CHECK:            updateTiltCheck();            break;
    case SUSTAINER_IGNITION:    updateSustainerIgnition();    break;
    case STAGE2_BOOST:          updateStage2Boost();          break;
    case COAST:                 updateCoast();                break;
    case POST_APOGEE_DESCENT:   updatePostApogeeDescent();    break;
    case MAIN_FIRED:            updateMainFired();            break;
    case DESCENT_UNDER_MAIN:    updateDescentUnderMain();     break;
    case TOUCHDOWN:             updateTouchdown();            break;
    case ABORT_DESCENT_DROGUE:  updateAbortDescentDrogue();   break;
    case ABORT_MAIN_FIRED:      updateAbortMainFired();       break;
    case ABORT_DESCENT_MAIN:    updateAbortDescentMain();     break;
    case ABORT_TOUCHDOWN:       updateAbortTouchdown();       break;
  }
}

// =============================================================================
// PRE_LIFTOFF — wait for booster ignition (sustainer is hot-staged, so it
// experiences the booster's accel as its own "liftoff" signal)
// =============================================================================
void SustainerFlightStrategy::updatePreLiftoff() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (accel > Config::Safety::LIFTOFF_ACCEL_MPS2) {
    if (_liftoffCandidateMs == 0) _liftoffCandidateMs = now;
    if ((now - _liftoffCandidateMs) >= Config::Safety::LIFTOFF_DURATION_MS) {
      enterStage1Boost();
    }
  } else {
    _liftoffCandidateMs = 0;
  }
}

void SustainerFlightStrategy::enterStage1Boost() {
  _liftoffMs = millis();
  _state = STAGE1_BOOST;
  _stage1BurnoutCandidateMs = 0;

  // Reset BOTH estimators at liftoff so they start from a clean state.
  // The StateEstimator zeros altitude/velocity. The TiltEstimator zeros
  // the orientation quaternion (assumes rocket is currently vertical).
  _estimator.reset();
  _tiltEst.reset();

  _uiBuzzer.beepOnce(100);
}

// =============================================================================
// STAGE1_BOOST — booster burning
// =============================================================================
void SustainerFlightStrategy::updateStage1Boost() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (fabsf(accel) < Config::Safety::BURNOUT_ACCEL_MPS2) {
    if (_stage1BurnoutCandidateMs == 0) _stage1BurnoutCandidateMs = now;
    if ((now - _stage1BurnoutCandidateMs) >= Config::Safety::BURNOUT_DURATION_MS) {
      enterSeparationCoast();
    }
  } else {
    _stage1BurnoutCandidateMs = 0;
  }
}

void SustainerFlightStrategy::enterSeparationCoast() {
  _stage1BurnoutMs = millis();
  _state = SEPARATION_COAST;
  _uiBuzzer.beepOnce(100);
}

// =============================================================================
// SEPARATION_COAST — waiting for ignition window
// =============================================================================
// Wait for SEPARATION_DELAY_MS after booster burnout. During this time:
//   - Physical separation occurs (sustainer pulls away from booster)
//   - Booster drogue deploys from the now-open interstage
//   - Rocket stabilizes for the tilt measurement
//
// The rate-damping controller is still DISABLED here: sustainer is still
// attached to booster, and the combined-vehicle dynamics are wrong for a
// sustainer-tuned controller.
// =============================================================================
void SustainerFlightStrategy::updateSeparationCoast() {
  uint32_t elapsedSinceBurnout = millis() - _stage1BurnoutMs;
  if (elapsedSinceBurnout >= Config::Safety::SEPARATION_DELAY_MS) {
    enterTiltCheck();
  }
}

void SustainerFlightStrategy::enterTiltCheck() {
  _state = TILT_CHECK;
  _tiltCheckStartMs = millis();
  // Two short beeps: tilt check decision point
  _uiBuzzer.beepPattern(2, 80, 80, 0);
}

// =============================================================================
// TILT_CHECK — decision moment: ignite or abort
// =============================================================================
void SustainerFlightStrategy::updateTiltCheck() {
  // Record diagnostic info
  _tiltAtIgnitionDeg = _tiltEst.tiltDeg();

  uint32_t elapsedInTiltCheck = millis() - _tiltCheckStartMs;

  if (_tiltEst.isWithinSafeCone()) {
    // Within safe cone — commit to ignition
    enterSustainerIgnition();
    return;
  }

  if (elapsedInTiltCheck >= Config::Safety::SUSTAINER_IGN_WINDOW_MS) {
    // Window expired without a safe-cone reading — abort
    enterAbortDescentDrogue();
  }
}

void SustainerFlightStrategy::enterSustainerIgnition() {
  _ignitionMs = millis();
  _state = SUSTAINER_IGNITION;

  // Fire the sustainer ignitor pyro
  _ignitorPyro.setArmed(true);
  _ignitorPyro.fire(Config::Pyro::FIRE_TIME_MS);

  // Three beeps: sustainer ignition event
  _uiBuzzer.beepPattern(3, 50, 80, 0);
}

// =============================================================================
// SUSTAINER_IGNITION — waiting for sustainer motor to catch
// =============================================================================
void SustainerFlightStrategy::updateSustainerIgnition() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (accel > Config::Safety::LIFTOFF_ACCEL_MPS2) {
    if (_stage2LiftoffCandidateMs == 0) _stage2LiftoffCandidateMs = now;
    if ((now - _stage2LiftoffCandidateMs) >= Config::Safety::LIFTOFF_DURATION_MS) {
      enterStage2Boost();
      return;
    }
  } else {
    _stage2LiftoffCandidateMs = 0;
  }

  // Timeout: if no ignition within 2s, treat as misfire and abort
  if ((now - _ignitionMs) > 2000) {
    enterAbortDescentDrogue();
  }
}

void SustainerFlightStrategy::enterStage2Boost() {
  _state = STAGE2_BOOST;
  _stage2BurnoutCandidateMs = 0;
  _uiBuzzer.beepOnce(100);

  // ===== Rate-damping controller ENABLED here =====
  // Sustainer thrust is confirmed, separation has occurred via the
  // ignition transient. The rocket is now alone with sustainer-only
  // dynamics — the regime the controller was tuned for. Controller
  // remains enabled through COAST until apogee.
  _rateDamper.enableControl();
}

// =============================================================================
// STAGE2_BOOST — sustainer burning
// =============================================================================
void SustainerFlightStrategy::updateStage2Boost() {
  float accel = _accelSrc.getAxialAccelMps2();
  uint32_t now = millis();

  if (fabsf(accel) < Config::Safety::BURNOUT_ACCEL_MPS2) {
    if (_stage2BurnoutCandidateMs == 0) _stage2BurnoutCandidateMs = now;
    if ((now - _stage2BurnoutCandidateMs) >= Config::Safety::BURNOUT_DURATION_MS) {
      enterCoast();
    }
  } else {
    _stage2BurnoutCandidateMs = 0;
  }
}

void SustainerFlightStrategy::enterCoast() {
  _stage2BurnoutMs = millis();
  _state = COAST;
  _uiBuzzer.beepOnce(100);
  // Controller remains ENABLED through coast — this is the primary regime
  // for rate damping (no thrust, real disturbances, good fin authority).
}

// =============================================================================
// COAST — waiting for apogee (motor ejection mechanically deploys drogue)
// =============================================================================
void SustainerFlightStrategy::updateCoast() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();

  if (altitude > _maxAltitudeSeen) {
    _maxAltitudeSeen = altitude;
  }

  bool descending = (velocity < 0.0f) && (altitude < _maxAltitudeSeen - 10.0f);

  if (descending) {
    enterPostApogeeDescent();
  }
}

void SustainerFlightStrategy::enterPostApogeeDescent() {
  _state = POST_APOGEE_DESCENT;
  _uiBuzzer.beepPattern(2, 80, 80, 0);

  // ===== Rate-damping controller DISABLED here =====
  // Fins are ineffective during descent under drogue (low velocity, bad
  // angle of attack relative to airflow). Center them and stop computing
  // commands — disableControl() also writes 90° to all servos.
  _rateDamper.disableControl();
}

// =============================================================================
// POST_APOGEE_DESCENT — falling under drogue, waiting for 1,500 ft
// =============================================================================
void SustainerFlightStrategy::updatePostApogeeDescent() {
  float altitude = _estimator.altitude_m();
  if (altitude < Config::Safety::MAIN_DEPLOY_ALT_M) {
    enterMainFired();
  }
}

void SustainerFlightStrategy::enterMainFired() {
  _mainFiredMs = millis();
  _state = MAIN_FIRED;
  _mainPyro.setArmed(true);
  _mainPyro.fire(Config::Pyro::FIRE_TIME_MS);
  _uiBuzzer.beepPattern(3, 50, 80, 0);
}

void SustainerFlightStrategy::updateMainFired() {
  if (!_mainPyro.isFiring()) {
    enterDescentUnderMain();
  }
}

void SustainerFlightStrategy::enterDescentUnderMain() {
  _state = DESCENT_UNDER_MAIN;
  _touchdownCandidateMs = 0;
}

void SustainerFlightStrategy::updateDescentUnderMain() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();
  uint32_t now = millis();

  bool slowEnough = (fabsf(velocity) < Config::Safety::TOUCHDOWN_VEL_MAX_MPS);
  bool lowEnough  = (altitude < Config::Safety::TOUCHDOWN_ALT_MAX_M);

  if (slowEnough && lowEnough) {
    if (_touchdownCandidateMs == 0) _touchdownCandidateMs = now;
    if ((now - _touchdownCandidateMs) >= Config::Safety::TOUCHDOWN_DURATION_MS) {
      enterTouchdown();
    }
  } else {
    _touchdownCandidateMs = 0;
  }
}

void SustainerFlightStrategy::enterTouchdown() {
  _touchdownMs = millis();
  _state = TOUCHDOWN;
  _recoveryBuzzer.setContinuous(true);
}

void SustainerFlightStrategy::updateTouchdown() {
  // Terminal state.
}

// =============================================================================
// ABORT BRANCH
// =============================================================================
// Tilt check failed or sustainer didn't ignite. Per team analysis:
//   - The interstage is friction-fit, so the rocket likely separates during
//     descent due to drag.
//   - Each stage's main parachute can still deploy at 1,500 ft.
//   - Recovery is degraded but probably survivable.
//
// The firmware's job in ABORT is to behave normally for descent and main
// deploy. We just never fire the sustainer ignitor. The rate-damping
// controller is forcibly disabled on each abort entry (defensive — it
// shouldn't have been enabled on the abort path in the first place).
// =============================================================================

void SustainerFlightStrategy::enterAbortDescentDrogue() {
  _state = ABORT_DESCENT_DROGUE;
  // Five rapid beeps: distinctive ABORT signal
  _uiBuzzer.beepPattern(5, 40, 40, 200);

  // Defensive disable — abort path should never have enabled it.
  _rateDamper.disableControl();
}

void SustainerFlightStrategy::updateAbortDescentDrogue() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();

  if (altitude > _maxAltitudeSeen) {
    _maxAltitudeSeen = altitude;
  }

  // Wait until we're actually descending past peak, then watch for 1,500 ft
  bool descending = (velocity < 0.0f) && (altitude < _maxAltitudeSeen - 10.0f);

  if (descending && altitude < Config::Safety::MAIN_DEPLOY_ALT_M) {
    enterAbortMainFired();
  }
}

void SustainerFlightStrategy::enterAbortMainFired() {
  _mainFiredMs = millis();
  _state = ABORT_MAIN_FIRED;
  _mainPyro.setArmed(true);
  _mainPyro.fire(Config::Pyro::FIRE_TIME_MS);

  _rateDamper.disableControl();
}

void SustainerFlightStrategy::updateAbortMainFired() {
  if (!_mainPyro.isFiring()) {
    enterAbortDescentMain();
  }
}

void SustainerFlightStrategy::enterAbortDescentMain() {
  _state = ABORT_DESCENT_MAIN;
  _touchdownCandidateMs = 0;

  _rateDamper.disableControl();
}

void SustainerFlightStrategy::updateAbortDescentMain() {
  float velocity = _estimator.velocity_mps();
  float altitude = _estimator.altitude_m();
  uint32_t now = millis();

  bool slowEnough = (fabsf(velocity) < Config::Safety::TOUCHDOWN_VEL_MAX_MPS);
  bool lowEnough  = (altitude < Config::Safety::TOUCHDOWN_ALT_MAX_M);

  if (slowEnough && lowEnough) {
    if (_touchdownCandidateMs == 0) _touchdownCandidateMs = now;
    if ((now - _touchdownCandidateMs) >= Config::Safety::TOUCHDOWN_DURATION_MS) {
      enterAbortTouchdown();
    }
  } else {
    _touchdownCandidateMs = 0;
  }
}

void SustainerFlightStrategy::enterAbortTouchdown() {
  _touchdownMs = millis();
  _state = ABORT_TOUCHDOWN;
  _recoveryBuzzer.setContinuous(true);

  _rateDamper.disableControl();
}

void SustainerFlightStrategy::updateAbortTouchdown() {
  // Terminal state.
}