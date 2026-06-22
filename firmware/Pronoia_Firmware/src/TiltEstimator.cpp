// =============================================================================
// TiltEstimator.cpp
// =============================================================================
#include "TiltEstimator.h"
#include "IMU.h"
#include <math.h>

TiltEstimator::TiltEstimator(IMU& imu) : _imu(imu) {
}

void TiltEstimator::reset() {
  // Identity quaternion = no rotation. The rocket's current orientation
  // becomes the reference. This should be called when the rocket is
  // vertical on the pad.
  _q[0] = 1.0f;
  _q[1] = 0.0f;
  _q[2] = 0.0f;
  _q[3] = 0.0f;

  _tiltDeg = 0.0f;
  _maxTiltDeg = 0.0f;
  _rotationFault = false;
  _lastUpdateMicros = 0;
  _initialized = false;
}

void TiltEstimator::update() {
  uint32_t nowMicros = micros();

  if (!_initialized) {
    _lastUpdateMicros = nowMicros;
    _initialized = true;
    return;
  }

  // Time delta in seconds since last update
  float dt = (nowMicros - _lastUpdateMicros) * 1.0e-6f;
  _lastUpdateMicros = nowMicros;

  // Guard against pathological dt (overflow, scheduling hiccup, etc.)
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  // ===== Read gyro rates (degrees per second), convert to radians per sec =====
  // Per IMU convention:
  //   Roll  = rotation about axial axis (rocket length)
  //   Pitch = rotation about Lat-1 axis
  //   Yaw   = rotation about Lat-2 axis
  // Body-frame angular velocity vector ωx, ωy, ωz where x is "first
  // lateral", y is axial, z is "second lateral".
  constexpr float DEG2RAD = 3.14159265358979f / 180.0f;
  float wx = _imu.pitchRateDps() * DEG2RAD;  // body-frame x rotation rate
  float wy = _imu.rollRateDps()  * DEG2RAD;  // body-frame y (axial) rotation rate
  float wz = _imu.yawRateDps()   * DEG2RAD;  // body-frame z rotation rate

  // ===== Quaternion derivative from body-frame angular velocity =====
  // Standard formula for q-dot when angular velocity is in body frame:
  //   q_dot = 0.5 * q ⊗ ω_quat
  // where ω_quat = [0, ωx, ωy, ωz] is the angular velocity expressed as
  // a pure quaternion.
  //
  // Expanded out (q = [w, x, y, z]):
  //   q_dot_w = 0.5 * (-x*wx - y*wy - z*wz)
  //   q_dot_x = 0.5 * ( w*wx + y*wz - z*wy)
  //   q_dot_y = 0.5 * ( w*wy - x*wz + z*wx)
  //   q_dot_z = 0.5 * ( w*wz + x*wy - y*wx)
  float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];

  float qDot_w = 0.5f * (-qx * wx - qy * wy - qz * wz);
  float qDot_x = 0.5f * ( qw * wx + qy * wz - qz * wy);
  float qDot_y = 0.5f * ( qw * wy - qx * wz + qz * wx);
  float qDot_z = 0.5f * ( qw * wz + qx * wy - qy * wx);

  // ===== Integrate (Euler) =====
  _q[0] += qDot_w * dt;
  _q[1] += qDot_x * dt;
  _q[2] += qDot_y * dt;
  _q[3] += qDot_z * dt;

  // ===== Renormalize quaternion =====
  // Integration accumulates small errors that grow the quaternion's magnitude.
  // We normalize to keep |q| = 1, which preserves the "unit quaternion"
  // property required for representing rotations.
  float qNorm = sqrtf(_q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]);
  if (qNorm > 1e-9f) {
    float invNorm = 1.0f / qNorm;
    _q[0] *= invNorm;
    _q[1] *= invNorm;
    _q[2] *= invNorm;
    _q[3] *= invNorm;
  }

  // ===== Compute tilt and update fault latch =====
  computeTilt();
}

void TiltEstimator::computeTilt() {
  // Compute the angle between:
  //   - The body-frame axial axis (rocket "up" in body frame) = [0, 1, 0]
  //   - The world-frame vertical axis (world "up") = [0, 1, 0] originally
  //
  // At reset, body frame == world frame, so axial axis aligned with vertical.
  // As the rocket rotates, the body-frame axial axis (in world coordinates)
  // becomes wherever the quaternion rotates [0, 1, 0] to.
  //
  // Equivalently, in the body frame, the world-vertical direction is
  // wherever the inverse rotation takes [0, 1, 0]. Either viewpoint
  // gives the same tilt angle.
  //
  // We'll compute "rocket's axial axis in world coordinates":
  //   v_world = R(q) * [0, 1, 0]
  //
  // From standard quaternion-to-matrix conversion, the rotation of [0, 1, 0]
  // by quaternion q = [w, x, y, z] is:
  //   v_world.x = 2*(xy + wz)
  //   v_world.y = 1 - 2*(x² + z²)        ← this is the dot product with world-Y
  //   v_world.z = 2*(yz - wx)
  //
  // We only need v_world.y because that's already the dot product with
  // world-vertical [0, 1, 0]. That dot product equals cos(tilt).
  float qx = _q[1], qz = _q[3];

  float cosTilt = 1.0f - 2.0f * (qx*qx + qz*qz);

  // Clamp to valid range for acosf (small numerical errors can push it past ±1)
  if (cosTilt >  1.0f) cosTilt =  1.0f;
  if (cosTilt < -1.0f) cosTilt = -1.0f;

  constexpr float RAD2DEG = 180.0f / 3.14159265358979f;
  _tiltDeg = acosf(cosTilt) * RAD2DEG;

  // Track maximum
  if (_tiltDeg > _maxTiltDeg) {
    _maxTiltDeg = _tiltDeg;
  }

  // ===== Latch rotation fault =====
  // Once we've gone beyond the safe cone, even briefly, the rotation fault
  // is permanent until the next reset(). This prevents sustainer ignition
  // even if the rocket happens to be within the cone right at decision time.
  if (_tiltDeg > MAX_TILT_DEG) {
    _rotationFault = true;
  }
}