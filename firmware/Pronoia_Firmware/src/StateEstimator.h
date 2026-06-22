// =============================================================================
// StateEstimator.h
// =============================================================================
// Complementary filter for altitude and velocity estimation.
//
// FUSES:
//   - AccelerationSource (gravity-removed axial acceleration, 1000 Hz)
//   - Barometer (smoothed altitude + finite-difference velocity, 50 Hz)
//
// PRODUCES:
//   - Fused altitude estimate (m AGL)
//   - Fused velocity estimate (m/s, positive = ascending)
//   - Both updated at the IMU rate, with baro corrections blended in
//     whenever a fresh baro sample is available AND sanity gates pass.
//
// WHY COMPLEMENTARY FILTER:
//   Accelerometer integration gives low-noise, low-phase-lag estimates over
//   short timescales but drifts over long timescales due to bias.
//   Barometer gives drift-free absolute reference but is noisy and phase-lagged.
//   The complementary filter trusts each sensor where it's strong:
//     - High-pass on accel integration (rejects drift)
//     - Low-pass on baro (rejects noise)
//   Crossover frequency set by alpha values below.
//
// SANITY GATES (when NOT to blend in baro):
//   - Baro velocity above 300 m/s — likely Mach transient artifact
//   - Altitude above 13,000m — ISA model degrades
//   - During boost (high accel) — accel is reliable, baro is suspect
//   The estimator coasts on accelerometer alone when baro is suspect, then
//   re-anchors to baro when conditions return to normal.
// =============================================================================

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <Arduino.h>

class AccelerationSource;
class Barometer;

class StateEstimator {
  public:
    StateEstimator(AccelerationSource& accelSrc, Barometer& baro);

    // Call once per main loop iteration AFTER accelSrc.update() and baro.update().
    void update();

    // ===== Fused state estimates =====
    // Best-estimate altitude above ground level, in meters.
    float altitude_m() const { return _altitude_m; }

    // Best-estimate velocity (positive = ascending), in m/s.
    float velocity_mps() const { return _velocity_mps; }

    // Pass-through of accel for telemetry convenience.
    float acceleration_mps2() const { return _accel_mps2; }

    // ===== Tracked extrema =====
    // Maximum altitude reached during flight (resets only via reset()).
    float maxAltitude_m() const { return _maxAltitude_m; }

    // Maximum velocity reached during flight.
    float maxVelocity_mps() const { return _maxVelocity_mps; }

    // ===== Diagnostics =====
    // Returns true when the last update() iteration blended in a baro sample.
    bool baroAccepted() const { return _baroAccepted; }

    // Returns true when sanity gates rejected the last baro sample (we coasted
    // on accelerometer alone). Useful for telemetry / post-flight analysis.
    bool baroRejected() const { return _baroRejected; }

    // ===== Reset =====
    // Resets all state to zero. Call once at liftoff detection so the
    // integrator starts clean. The integrators accumulate small errors during
    // pre-flight sitting on the pad; resetting at liftoff ensures only flight
    // data contributes to the estimate.
    void reset();

  private:
    AccelerationSource& _accelSrc;
    Barometer& _baro;

    // ===== Fused state =====
    float _altitude_m    = 0.0f;
    float _velocity_mps  = 0.0f;
    float _accel_mps2    = 0.0f;

    // ===== Tracked extrema =====
    float _maxAltitude_m   = 0.0f;
    float _maxVelocity_mps = 0.0f;

    // ===== Timing =====
    uint32_t _lastUpdateMicros = 0;
    bool _initialized = false;

    // ===== Baro freshness tracking =====
    // TODO: We blend baro only when a NEW baro sample is available. We detect that
    // by tracking the baro's maxAltitude_AGL — if it changes, or if the
    // smoothedAltitude_AGL has moved meaningfully, the baro has updated.
    // Cleaner: we'd add a baro.newSampleAvailable() flag. For now, we just
    // remember the last smoothed altitude we saw and detect changes.
    bool  _baroAccepted        = false;
    bool  _baroRejected        = false;

    // ===== Filter tuning =====
    // alpha_v: velocity filter — closer to 1.0 = trust accel more.
    // alpha_a: altitude filter — closer to 1.0 = trust accel-integrated more.
    // For tau_v = 1.0 sec, baro period = 20ms:  alpha = exp(-0.02/1.0) ≈ 0.98
    // For tau_a = 0.5 sec, baro period = 20ms:  alpha = exp(-0.02/0.5) ≈ 0.96
    // Lower alpha = more baro trust = less drift but more noise/lag.
    static constexpr float ALPHA_VELOCITY = 0.98f;
    static constexpr float ALPHA_ALTITUDE = 0.96f;

    // Sanity gates for blending baro
    static constexpr float BARO_VEL_TRUST_MAX_MPS = 300.0f;
    static constexpr float BARO_ALT_TRUST_MAX_M   = 5000.0f;
    static constexpr float COAST_ACCEL_THRESHOLD_MPS2 = 0.2f * 9.80665f;
};

#endif