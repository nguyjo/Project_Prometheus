// =============================================================================
// RateDampingController.cpp
// =============================================================================
// See header for design intent and gate philosophy.
// =============================================================================

#include "RateDampingController.h"
#include "IMU.h"
#include "Barometer.h"
#include "StateEstimator.h"
#include "Config.h"
#include <math.h>

// File-local helper, in anonymous namespace to avoid any possibility of
// collision with same-named symbols in other translation units.
namespace {
    int rawClampDeg(int deg) {
        const int lo = (int)(90.0f - RateDampingController::MAX_SERVO_DEFLECT_DEG);
        const int hi = (int)(90.0f + RateDampingController::MAX_SERVO_DEFLECT_DEG);
        if (deg < lo) deg = lo;
        if (deg > hi) deg = hi;
        return deg;
    }
}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
RateDampingController::RateDampingController(
        IMU& imu, Barometer& baro, StateEstimator& estimator,
        int servoAPin, int servoBPin, int servoCPin, int servoDPin)
    : _imu(imu), _baro(baro), _estimator(estimator),
      _servoAPin(servoAPin), _servoBPin(servoBPin),
      _servoCPin(servoCPin), _servoDPin(servoDPin)
{}

// -----------------------------------------------------------------------------
// begin() — attach servos and command neutral.
// -----------------------------------------------------------------------------
void RateDampingController::begin() {
    _servoA.attach(_servoAPin);
    _servoB.attach(_servoBPin);
    _servoC.attach(_servoCPin);
    _servoD.attach(_servoDPin);
    centerAllFins();
}

// -----------------------------------------------------------------------------
// centerAllFins() — write 90° to all servos.
// -----------------------------------------------------------------------------
void RateDampingController::centerAllFins() {
    _cmdA = _cmdB = _cmdC = _cmdD = 90.0f;
    writeServos(_cmdA, _cmdB, _cmdC, _cmdD);
}

// -----------------------------------------------------------------------------
// setTrim()
// -----------------------------------------------------------------------------
void RateDampingController::setTrim(int a, int b, int c, int d) {
    _trimA = a; _trimB = b; _trimC = c; _trimD = d;
}

// -----------------------------------------------------------------------------
// setTrimOne() — set one servo's trim with validation.
// -----------------------------------------------------------------------------
bool RateDampingController::setTrimOne(char fin, int trimDeg) {
    if (trimDeg < -MAX_TRIM_DEG || trimDeg > MAX_TRIM_DEG) {
        return false;
    }
    switch (fin) {
        case 'A': _trimA = trimDeg; return true;
        case 'B': _trimB = trimDeg; return true;
        case 'C': _trimC = trimDeg; return true;
        case 'D': _trimD = trimDeg; return true;
        default:  return false;
    }
}

// -----------------------------------------------------------------------------
// writeServos() — apply trim, clamp to mechanical range, write.
// -----------------------------------------------------------------------------
void RateDampingController::writeServos(float cA, float cB, float cC, float cD) {
    _servoA.write(rawClampDeg((int)(cA - _trimA + 0.5f)));
    _servoB.write(rawClampDeg((int)(cB - _trimB + 0.5f)));
    _servoC.write(rawClampDeg((int)(cC - _trimC + 0.5f)));
    _servoD.write(rawClampDeg((int)(cD - _trimD + 0.5f)));
}

// -----------------------------------------------------------------------------
// writeRawServoX() — for bench-test CLI. Writes one servo directly.
// Applies the same mechanical-limit clamp as the control loop. Trim is NOT
// applied here — bench test characterizes raw servo behavior pre-trim.
// -----------------------------------------------------------------------------
void RateDampingController::writeRawServoA(int deg) { _servoA.write(rawClampDeg(deg)); _cmdA = (float)deg; }
void RateDampingController::writeRawServoB(int deg) { _servoB.write(rawClampDeg(deg)); _cmdB = (float)deg; }
void RateDampingController::writeRawServoC(int deg) { _servoC.write(rawClampDeg(deg)); _cmdC = (float)deg; }
void RateDampingController::writeRawServoD(int deg) { _servoD.write(rawClampDeg(deg)); _cmdD = (float)deg; }

// -----------------------------------------------------------------------------
// rotateImuRatesToAirframe()
// -----------------------------------------------------------------------------
void RateDampingController::rotateImuRatesToAirframe(
        float pitchRate_imu, float yawRate_imu,
        float& pitchRate_af, float& yawRate_af) const {
    constexpr float theta_rad = BODY_ROTATION_OFFSET_DEG * (float)M_PI / 180.0f;
    const float c = cosf(theta_rad);
    const float s = sinf(theta_rad);
    pitchRate_af =  pitchRate_imu * c + yawRate_imu * s;
    yawRate_af   = -pitchRate_imu * s + yawRate_imu * c;
}

// -----------------------------------------------------------------------------
// computeDragForce()
// -----------------------------------------------------------------------------
float RateDampingController::computeDragForce(float velocity_mps) const {
    const float pressure_Pa = _baro.pressure_Mbar * 100.0f;
    const float temp_K      = _baro.temp_C + 273.15f;
    const float airDensity  = pressure_Pa / (R_DRY_AIR * temp_K);
    float dragForce = 0.5f * CTRL_SURFACE_CD * velocity_mps * velocity_mps
                      * CTRL_SURFACE_AREA_M2 * airDensity;
    if (dragForce < MIN_DRAG_FORCE_N) dragForce = MIN_DRAG_FORCE_N;
    return dragForce;
}

// -----------------------------------------------------------------------------
// update() — the control loop.
// -----------------------------------------------------------------------------
void RateDampingController::update() {
    // --- Bench-suspend takes priority over everything ---
    if (_benchSuspend) {
        return;  // raw servo writes are managed by CLI; don't touch them
    }

    // =========================================================================
    // NEUTRAL FIN MODE — SAFETY PATCH (Launch Window — May 2026)
    // All four servos held at trim-adjusted neutral for entire flight.
    // Active rate-damping control DISABLED. Rocket flies as a passive
    // fin-stabilized vehicle.
    //
    // - Bench-suspend (above) still takes priority so CANARD_TEST works.
    // - centerAllFins() writes 90° via writeServos(), which applies trim.
    // - Per-servo trims set via CANARD_TRIM_SET CLI before flight.
    //
    // TO RESTORE ACTIVE CONTROL: delete this block.
    // =========================================================================
    centerAllFins();
    return;

    // --- Rate-limit to CONTROL_HZ ---
    const uint32_t now = micros();
    const uint32_t targetPeriod_us = 1000000UL / Config::Rates::CONTROL_HZ;
    if (!_firstUpdate && (now - _lastUpdateMicros) < targetPeriod_us) {
        return;
    }
    const float dt_s = _firstUpdate ? (1.0f / (float)Config::Rates::CONTROL_HZ)
                                    : (now - _lastUpdateMicros) * 1e-6f;
    _lastUpdateMicros = now;
    _firstUpdate = false;

    // --- Authority gates ---
    if (!_controlEnabled) {
        centerAllFins();
        return;
    }

    float velocity = _estimator.velocity_mps();
    if (_benchTestMode) {
        velocity = BENCH_TEST_VELOCITY_MPS;
    } else if (velocity < MIN_CONTROL_VELOCITY_MPS) {
        centerAllFins();
        return;
    }

    // --- Read gyro rates (deg/s, IMU body frame) ---
    const float rollRate      = _imu.rollRateDps();
    const float pitchRate_imu = _imu.pitchRateDps();
    const float yawRate_imu   = _imu.yawRateDps();

    // --- Rotate pitch/yaw into airframe frame ---
    float pitchRate, yawRate;
    rotateImuRatesToAirframe(pitchRate_imu, yawRate_imu, pitchRate, yawRate);

    // --- Error = setpoint (0) - measurement ---
    const float rollErr  = -rollRate;
    const float pitchErr = -pitchRate;
    const float yawErr   = -yawRate;

    // --- Derivative of error ---
    const float dRoll_dt  = (rollErr  - _prevRollErr)  / dt_s;
    const float dPitch_dt = (pitchErr - _prevPitchErr) / dt_s;
    const float dYaw_dt   = (yawErr   - _prevYawErr)   / dt_s;
    _prevRollErr  = rollErr;
    _prevPitchErr = pitchErr;
    _prevYawErr   = yawErr;

    // --- PD law: torque demand per axis ---
    float rollTorque  = KP_ROLL  * rollErr  + KD_ROLL  * dRoll_dt;
    float pitchTorque = KP_PITCH * pitchErr + KD_PITCH * dPitch_dt;
    float yawTorque   = KP_YAW   * yawErr   + KD_YAW   * dYaw_dt;

    // --- Clamp per-axis torque ---
    auto clampTorque = [](float t) -> float {
        if (t >  MAX_AXIS_TORQUE_NM) return  MAX_AXIS_TORQUE_NM;
        if (t < -MAX_AXIS_TORQUE_NM) return -MAX_AXIS_TORQUE_NM;
        return t;
    };
    rollTorque  = clampTorque(rollTorque);
    pitchTorque = clampTorque(pitchTorque);
    yawTorque   = clampTorque(yawTorque);

    // --- Compute drag force per fin ---
    const float dragForce = computeDragForce(velocity);
    _lastDragForce = dragForce;

    const float ypArm   = YAWPIT_TORQUE_ARM_M;
    const float rollArm = ROLL_TORQUE_ARM_M;

    // --- Sine demands, declared at function scope so both branches and the
    //     shared clamp/convert/write sequence below can see them. ---
    float sineA = 0.0f;
    float sineB = 0.0f;
    float sineC = 0.0f;
    float sineD = 0.0f;

    if constexpr (Config::Control::TWO_FIN_MODE) {
        // =====================================================================
        // EMERGENCY 2-FIN MODE: only B and D active.
        // - Pitch torque distributed across B and D (anti-symmetric, ypArm).
        // - Yaw is NOT commanded (no Y-axis fins available).
        // - Roll is NOT commanded — see rationale below.
        // - A and C are forced to center (90 deg).
        //
        // WHY NO ROLL IN 2-FIN MODE:
        //   With only B and D, a "same-sign" deflection in both fins produces
        //   roll torque about the long axis. But because B and D are on
        //   opposite ends of the Z-axis diameter (not the centerline), the
        //   aerodynamic forces couple into pitch under any non-axial flow
        //   condition (gust, AOA, off-vertical climb). The roll loop and
        //   pitch loop would fight each other. Passive fin stability handles
        //   long-axis rotation acceptably; we accept whatever roll rate the
        //   rocket has and damp pitch only.
        // =====================================================================

        // Pitch torque: anti-symmetric across B and D.
        const float pitchPerFin = pitchTorque / 2.0f;

        // Per-fin sine demands for pitch (B at Z-, D at Z+).
        const float sineAxisB = -pitchPerFin / (ypArm * dragForce);
        const float sineAxisD =  pitchPerFin / (ypArm * dragForce);

        // Combine on each fin, apply per-servo sign flag.
        // Roll authority intentionally zero. A and C stay at zero (center).
        sineB = (float)SIGN_B * sineAxisB;
        sineD = (float)SIGN_D * sineAxisD;
        // sineA and sineC remain 0.0f from declaration above.

    } else {
        // =====================================================================
        // ORIGINAL 4-FIN MODE
        // =====================================================================
        const float rollPerFin  = rollTorque  / 4.0f;
        const float pitchPerFin = pitchTorque / 2.0f;
        const float yawPerFin   = yawTorque   / 2.0f;

        const float sineRollA = rollPerFin / (rollArm * dragForce);
        const float sineRollB = rollPerFin / (rollArm * dragForce);
        const float sineRollC = rollPerFin / (rollArm * dragForce);
        const float sineRollD = rollPerFin / (rollArm * dragForce);

        float sineAxisA, sineAxisB, sineAxisC, sineAxisD;
        if (AC_IS_PITCH_PAIR) {
            sineAxisA =  pitchPerFin / (ypArm * dragForce);
            sineAxisC = -pitchPerFin / (ypArm * dragForce);
            sineAxisB =  yawPerFin   / (ypArm * dragForce);
            sineAxisD = -yawPerFin   / (ypArm * dragForce);
        } else {
            sineAxisA =  yawPerFin   / (ypArm * dragForce);
            sineAxisC = -yawPerFin   / (ypArm * dragForce);
            sineAxisB = -pitchPerFin / (ypArm * dragForce);
            sineAxisD =  pitchPerFin / (ypArm * dragForce);
        }

        sineA = (float)SIGN_A * (sineRollA + sineAxisA);
        sineB = (float)SIGN_B * (sineRollB + sineAxisB);
        sineC = (float)SIGN_C * (sineRollC + sineAxisC);
        sineD = (float)SIGN_D * (sineRollD + sineAxisD);
    }

    // --- Clamp to mechanical deflection limit (shared by both modes) ---
    const float maxSine = sinf(MAX_SERVO_DEFLECT_DEG * (float)M_PI / 180.0f);
    auto clampSine = [maxSine](float s) -> float {
        if (s >  maxSine) return  maxSine;
        if (s < -maxSine) return -maxSine;
        return s;
    };
    sineA = clampSine(sineA);
    sineB = clampSine(sineB);
    sineC = clampSine(sineC);
    sineD = clampSine(sineD);

    // --- Convert to servo angle ---
    const float rad2deg = 180.0f / (float)M_PI;
    _cmdA = 90.0f + asinf(sineA) * rad2deg;
    _cmdB = 90.0f + asinf(sineB) * rad2deg;
    _cmdC = 90.0f + asinf(sineC) * rad2deg;
    _cmdD = 90.0f + asinf(sineD) * rad2deg;

    _lastRollErr  = rollErr;
    _lastPitchErr = pitchErr;
    _lastYawErr   = yawErr;

    writeServos(_cmdA, _cmdB, _cmdC, _cmdD);
}