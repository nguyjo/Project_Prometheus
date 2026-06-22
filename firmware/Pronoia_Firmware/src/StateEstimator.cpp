// =============================================================================
// StateEstimator.cpp
// =============================================================================
#include "StateEstimator.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include <math.h>

StateEstimator::StateEstimator(AccelerationSource& accelSrc, Barometer& baro)
  : _accelSrc(accelSrc), _baro(baro) {
}

void StateEstimator::reset() {
  _altitude_m       = 0.0f;
  _velocity_mps     = 0.0f;
  _accel_mps2       = 0.0f;
  _maxAltitude_m    = 0.0f;
  _maxVelocity_mps  = 0.0f;
  _lastUpdateMicros = 0;
  _initialized      = false;
  _baroAccepted     = false;
  _baroRejected     = false;
}

void StateEstimator::update() {
  uint32_t nowMicros = micros();

  // First call: establish baseline and exit. No dt to integrate over yet.
  if (!_initialized) {
    _lastUpdateMicros = nowMicros;
    _initialized = true;
    _accel_mps2 = _accelSrc.getAxialAccelMps2();
    return;
  }

  // Compute dt since last call, in seconds.
  float dt = (nowMicros - _lastUpdateMicros) * 1.0e-6f;
  _lastUpdateMicros = nowMicros;

  // Guard against pathological dt values (millis() wraparound, scheduling
  // hiccups, etc). At nominal 1000 Hz, dt should be ~1ms. Clamp to a
  // reasonable range.
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  // ===== STEP 1: Read acceleration and integrate =====
  _accel_mps2 = _accelSrc.getAxialAccelMps2();

  // Trapezoidal integration would be more accurate, but at 1000 Hz the
  // difference is negligible and rectangular is simpler/faster. We're
  // integrating axial acceleration directly — _accel_mps2 is already
  // gravity-removed in the rocket frame (handled by AccelerationSource).
  //
  // Sign convention: positive axial accel = "felt-acceleration along the
  // rocket axis" = thrust pushing upward when vertical, equivalent to
  // upward motion.
  _velocity_mps += _accel_mps2 * dt;
  _altitude_m   += _velocity_mps * dt;

  // ===== STEP 2: Check for fresh baro sample =====
  // The Barometer sets an internal flag whenever it completes a new sample.
  // We consume the flag (read-and-clear) to avoid double-processing.
  bool baroIsFresh = _baro.consumeNewSampleFlag();
  _baroAccepted = false;
  _baroRejected = false;

  if (baroIsFresh) {
    // ===== STEP 3: Sanity-gate the baro sample =====
    // Reject baro if:
    //   - Baro velocity exceeds Mach-region trust threshold
    //   - Altitude exceeds ISA model trust threshold
    //   - We're in active boost (accel high) — accel is reliable, baro
    //     during boost can show pressure-spike artifacts.
    //
    // During reject conditions, we coast on accelerometer integration alone.
    // When conditions normalize, we re-blend baro and the integrator
    // gradually re-anchors to truth.
    bool baroVelOK = (fabsf(_baro.velocity_mps) < BARO_VEL_TRUST_MAX_MPS);
    bool baroAltOK = (_baro.smoothedAltitude_AGL < BARO_ALT_TRUST_MAX_M);
    bool inCoast   = (fabsf(_accel_mps2) < COAST_ACCEL_THRESHOLD_MPS2);

    if (baroVelOK && baroAltOK && inCoast) {
      // ===== STEP 4: Blend baro into the estimate (complementary filter) =====
      // Convention: alpha closer to 1 = trust accel-integrated estimate more.
      // (1 - alpha) = baro weight per fusion update.
      _velocity_mps = ALPHA_VELOCITY * _velocity_mps
                    + (1.0f - ALPHA_VELOCITY) * _baro.velocity_mps;
      _altitude_m   = ALPHA_ALTITUDE * _altitude_m
                    + (1.0f - ALPHA_ALTITUDE) * _baro.smoothedAltitude_AGL;
      _baroAccepted = true;
    } else {
      _baroRejected = true;
    }
  }

  // ===== STEP 5: Track extrema =====
  if (_altitude_m > _maxAltitude_m) {
    _maxAltitude_m = _altitude_m;
  }
  if (_velocity_mps > _maxVelocity_mps) {
    _maxVelocity_mps = _velocity_mps;
  }
}