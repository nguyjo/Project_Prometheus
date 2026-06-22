// =============================================================================
// TiltEstimator.h
// =============================================================================
// Tracks rocket orientation in 3D space using quaternion integration of gyro
// rates. Reports "tilt off vertical" — the angle between the rocket's body
// axial axis and the world's up direction.
//
// WHY QUATERNIONS:
//   Euler-angle integration suffers from gimbal lock and accumulating error
//   on arbitrary 3D rotations. Quaternions handle any rotation cleanly.
//   The math is more involved but the code isn't much longer.
//
// HOW IT WORKS:
//   1. At rest on the pad, the rocket is assumed vertical. The world-frame
//      up vector matches the rocket's body-frame axial axis. We initialize
//      the orientation quaternion to identity (no rotation).
//
//   2. Each IMU update, we integrate the gyro rates (deg/s) into the
//      quaternion. After many updates, the quaternion represents how the
//      rocket's body frame has rotated relative to its initial orientation.
//
//   3. To compute tilt off vertical: rotate the initial up vector by the
//      current orientation quaternion, and measure the angle between the
//      rotated vector and the rocket's body axial axis.
//
// AXIS CONVENTION (rocket body frame):
//   - Roll  = rotation about axial (rocket length) axis = IMU gyroY
//   - Pitch = rotation about Lat-1 axis                = IMU gyroX
//   - Yaw   = rotation about Lat-2 axis                = IMU gyroZ
//
// ROTATION FAULT LATCH:
//   Once tilt exceeds the maximum allowed value (MAX_TILT_DEG), set a
//   permanent fault flag. The flag never clears. The sustainer ignition
//   logic should check both "current tilt < max" AND "no rotation fault"
//   before firing.
//
//   This is the SparkyVT pattern: if the rocket ever goes beyond the
//   safe cone, even briefly, it's never safe to ignite the sustainer
//   regardless of where the rocket is right now.
//
// CALLER RESPONSIBILITIES:
//   - Call reset() when on the pad in a known vertical state, just before
//     liftoff. This initializes the quaternion to identity and clears the
//     rotation fault. Best practice: call from FlightStrategy at the moment
//     liftoff is latched.
//   - Call update() every IMU loop iteration (1000 Hz) so integration is
//     fine-grained enough for accurate angle tracking.
// =============================================================================

#ifndef TILTESTIMATOR_H
#define TILTESTIMATOR_H

#include <Arduino.h>

class IMU;

class TiltEstimator {
  public:
    TiltEstimator(IMU& imu);

    // Initialize: identity quaternion, no fault. Call when on the pad just
    // before liftoff (after the rocket is on its rail and vertical).
    void reset();

    // Integrate one gyro sample into the quaternion. Call every IMU update.
    void update();

    // Current angle off vertical, in degrees. Returns 0 when perfectly
    // vertical, 90 when horizontal, 180 when inverted.
    float tiltDeg() const { return _tiltDeg; }

    // Maximum tilt seen since last reset(). Useful for telemetry.
    float maxTiltDeg() const { return _maxTiltDeg; }

    // True if tilt has EVER exceeded the safe threshold since reset.
    // Latches once true — never clears until the next reset.
    bool rotationFault() const { return _rotationFault; }

    // Combined check: tilt within limits AND no historical fault.
    // This is what flight code should use as the sustainer ignition gate.
    bool isWithinSafeCone() const {
      return !_rotationFault && _tiltDeg < MAX_TILT_DEG;
    }

    // Diagnostic accessors
    float quaternionW() const { return _q[0]; }
    float quaternionX() const { return _q[1]; }
    float quaternionY() const { return _q[2]; }
    float quaternionZ() const { return _q[3]; }

  private:
    IMU& _imu;

    // Quaternion representing rocket-frame rotation relative to launch frame.
    // Format: [w, x, y, z]. Identity = [1, 0, 0, 0].
    float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // Most recently computed tilt angle (degrees off vertical)
    float _tiltDeg     = 0.0f;
    float _maxTiltDeg  = 0.0f;

    // Rotation fault latch — never clears until reset()
    bool _rotationFault = false;

    // Timing
    uint32_t _lastUpdateMicros = 0;
    bool _initialized = false;

    // ===== Configuration =====
    // Maximum tilt allowed before flagging rotation fault.
    // Per team requirements, set to 45° for Prometheus sustainer ignition.
    static constexpr float MAX_TILT_DEG = 45.0f;

    // Helper: compute tilt from current quaternion
    void computeTilt();
};

#endif