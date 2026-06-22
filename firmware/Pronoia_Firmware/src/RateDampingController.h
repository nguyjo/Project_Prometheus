// =============================================================================
// RateDampingController.h
// =============================================================================
// 3-axis angular rate damping using 4 canard servos (MEX-12).
//
// MISSION SCOPE — PURE RATE DAMPING:
//   This controller drives angular RATE to zero on all three body axes
//   (roll, pitch, yaw). It does NOT attempt to drive angular POSITION
//   to a setpoint. We are NOT guiding the rocket toward vertical.
//
//   Why this matters: a position-controlled "guide to vertical" rocket
//   would actively oppose wind weathercocking, which is what gives an
//   unguided rocket its passive stability. Fighting weathercocking can
//   create roll-pitch-yaw coupling and dynamically unstable conditions
//   if the gains are off. Rate damping just removes oscillations without
//   touching the natural ballistic trajectory.
//
// CONTROL LAW:
//   For each axis:  torque = Kp * (0 - rate) + Kd * dRate/dt
//   We don't use an integral term — there's no steady-state error to
//   integrate away (set-point is zero rate, and the rocket can't have
//   permanent steady-state angular rate without a stuck disturbance).
//
// FIN NAMING:
//   Mechanical team labels the four canard fins A, B, C, D going alphabetically
//   around the rocket. We use the same labels throughout firmware. NO numeric
//   "1/2/3/4" naming anywhere — single source of truth.
//
// FIN PAIR ASSIGNMENT (Prometheus 2025-2026):
//   A is at Y+, C is at Y-  →  A and C share the Y-axis diameter
//   B is at Z+, D is at Z-  →  B and D share the Z-axis diameter
//
//   A deflecting up + C deflecting down produces a couple that rotates
//   the rocket about the Z axis — that is YAW.
//   B deflecting up + D deflecting down produces a couple that rotates
//   the rocket about the Y axis — that is PITCH.
//
//   Therefore: AC = yaw pair, BD = pitch pair.
//   AC_IS_PITCH_PAIR is set to false below to reflect this.
//
// PER-SERVO SIGN CONVENTIONS:
//   Each servo's hardware may produce torque in either the "positive"
//   or "negative" sense for the same commanded angle. SIGN_A..D must be
//   set to +1 or -1 based on bench testing.
//
// PCBA-TO-AIRFRAME ROTATION OFFSET:
//   The Prometheus electronics bay is mechanically aligned to the airframe
//   via a coupler and shear pins (rotational alignment), with a tensioned
//   rod-and-nut clamping the sled (also rotational alignment when torqued
//   properly). Default BODY_ROTATION_OFFSET_DEG = 0. Verify alignment via
//   bench test 1 (described in the alignment doc) before flight.
//
// SAFETY GATES (the controller is INHIBITED — fins centered at 90° — when):
//   - Control not yet enabled by FlightStrategy
//   - Velocity below MIN_CONTROL_VELOCITY_MPS (no fin authority anyway)
//   - In bench-test mode, velocity check is bypassed
//
// NOTE ON FIN_CD: The flat-plate normal force model used here lumps
// the airfoil lift curve slope into a single "effective Cd". The true
// normal force coefficient for a low-AR thin fin at angle α is
// approximately Cl_α × α (with Cl_α ≈ 3–5 per radian), not Cd × sin(α).
// In practice, errors of factor 2–3 in this term are absorbed by
// KP_ROLL / KP_PITCH / KP_YAW during HIL tuning. Do NOT spend effort
// trying to derive an "exact" Cd; just tune the loop gain.
// =============================================================================

#ifndef RATEDAMPINGCONTROLLER_H
#define RATEDAMPINGCONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class IMU;
class Barometer;
class StateEstimator;

class RateDampingController {
  public:
    RateDampingController(IMU& imu,
                          Barometer& baro,
                          StateEstimator& estimator,
                          int servoAPin, int servoBPin,
                          int servoCPin, int servoDPin);

    void begin();
    void update();

    // Authority gating
    void enableControl()  { _controlEnabled = true; }
    void disableControl() { _controlEnabled = false; centerAllFins(); }
    void centerAllFins();

    // Trim offsets (degrees subtracted from commanded angle)
    void setTrim(int aTrim, int bTrim, int cTrim, int dTrim);

    // Read current trim values back (for CLI display)
    int trimA() const { return _trimA; }
    int trimB() const { return _trimB; }
    int trimC() const { return _trimC; }
    int trimD() const { return _trimD; }

    // Set a single trim. Returns false if value is outside ±MAX_TRIM_DEG.
    // trim is in degrees; commanded servo angle is reduced by this amount
    // before writing to the hardware (commanded = desired - trim).
    bool setTrimOne(char fin, int trimDeg);

    // Reset all trims to zero.
    void clearTrims() { _trimA = _trimB = _trimC = _trimD = 0; }

    // EXPANDED for neutral-fin mode launch (May 2026).
    // Standard limit (7) protects active-control authority budget.
    // In neutral-fin mode, the trim IS the position - no control authority
    // is being used, so the full mechanical travel is available for
    // cosmetic alignment.
    // RESTORE TO 7 when re-enabling active control.
    static constexpr int MAX_TRIM_DEG = 45;

    // Bench-suspend mode: controller skips its update() entirely. Used by the
    // CANARD_BENCH CLI command so writeRawServoX() commands are not overwritten
    // by the next control cycle. When toggled OFF, fins are centered and normal
    // operation resumes (but controller still gated by _controlEnabled).
    void setBenchSuspend(bool on) {
        _benchSuspend = on;
        if (on) centerAllFins();
}
bool isBenchSuspended() const { return _benchSuspend; }

    // ----- Bench-test single-servo CLI helpers -----
    // These bypass the controller math entirely and write a raw angle to one
    // servo. Useful for the CANARD_TEST CLI command — characterize each servo's
    // direction of deflection one at a time on the bench. NEVER call these
    // during flight mode; the controller will overwrite on the next update().
    void writeRawServoA(int deg);
    void writeRawServoB(int deg);
    void writeRawServoC(int deg);
    void writeRawServoD(int deg);

    // Diagnostic accessors
    float commandedDegA() const { return _cmdA; }
    float commandedDegB() const { return _cmdB; }
    float commandedDegC() const { return _cmdC; }
    float commandedDegD() const { return _cmdD; }
    float lastRollErrorDps()  const { return _lastRollErr; }
    float lastPitchErrorDps() const { return _lastPitchErr; }
    float lastYawErrorDps()   const { return _lastYawErr; }
    float lastDragForceN()    const { return _lastDragForce; }
    bool  isControlEnabled()  const { return _controlEnabled; }

    // =========================================================================
    // PID gains — tune in HIL
    // =========================================================================
    // In RateDampingController.h, conservative starting gains for HIL tuning.
    // Ratio reflects measured MOI asymmetry: I_pitch/I_roll ≈ 60×.
    static constexpr float KP_PITCH = 0.030f;
    static constexpr float KD_PITCH = 0.0010f;
    static constexpr float KP_YAW   = 0.030f;
    static constexpr float KD_YAW   = 0.0010f;
    static constexpr float KP_ROLL  = 0.0010f;  // ← much smaller than pitch/yaw
    static constexpr float KD_ROLL  = 0.0003f;

    // =========================================================================
    // FIN PAIR ASSIGNMENT (Prometheus 2025-2026 geometry)
    // =========================================================================
    // false: A+C is the YAW pair (Y-axis diameter), B+D is the PITCH pair
    // true:  A+C is the PITCH pair,                 B+D is the YAW pair
    //
    // Per the mechanical layout documented above:
    //   A=Y+, C=Y-  →  Y-axis diameter  →  yaw couple
    //   B=Z+, D=Z-  →  Z-axis diameter  →  pitch couple
    //
    // VERIFY VIA BENCH TEST: with one servo at a time wired and a fin tab
    // attached, command 90°→120° via CANARD_TEST CLI. Observe direction.
    // If a single A fin deflection rotates the rocket about Y (pitch),
    // not Z (yaw), then the convention is opposite and this flag must flip.
    static constexpr bool AC_IS_PITCH_PAIR = false;

    // =========================================================================
    // PER-SERVO SIGN — TODO: VERIFY VIA BENCH TEST BEFORE FLIGHT
    // =========================================================================
    // +1 = positive commanded throw produces positive control torque on
    //      that servo's assigned axis (yaw for A/C, pitch for B/D).
    // -1 = flip (servo mounted opposite, or horn keyed inverted).
    //
    // PLACEHOLDER VALUES — DO NOT TRUST WITHOUT BENCH TEST CONFIRMATION
    static constexpr int SIGN_A = +1;
    static constexpr int SIGN_B = +1;
    static constexpr int SIGN_C = +1;
    static constexpr int SIGN_D = +1;

    // =========================================================================
    // PCBA-TO-AIRFRAME ROTATION OFFSET
    // =========================================================================
    // Rotation of the IMU's body frame about the rocket's long axis,
    // relative to the fin frame, in degrees. Positive offset means the
    // PCBA is rotated counterclockwise (viewed from nose) relative to fins.
    //
    // Prometheus uses shear-pin coupler alignment + tensioned-rod sled
    // clamping. With proper assembly, this should be 0. Verify with bench
    // test 1 (pitch the rocket forward, see which gyro axis responds).
    static constexpr float BODY_ROTATION_OFFSET_DEG = 0.0f;

    // =========================================================================
    // Servo limits & gates
    // =========================================================================
    // MEX-12 servo at 8.4V (2S LiPo, Prometheus power budget):
    //   Max stall torque = 12.5 kg·cm = 1.226 N·m
    //   We clamp to 75% of stall = 0.92 N·m, leaving headroom against
    //   stall current spikes and motor controller cycling.
    //
    // If you ever switch to a regulated 6.0V servo rail, max torque drops
    // to ~0.78 N·m and this clamp should be lowered to ~0.59 N·m.
    static constexpr float MAX_AXIS_TORQUE_NM       = 0.92f;

    // EXPANDED for neutral-fin mode launch (May 2026).
    // Original 25° protects the active-control loop from commanding
    // angles where the servo horn binds against the sled.
    // In neutral-fin mode we statically hold position; expanded to 45°
    // to allow trim alignment of fins whose mechanical install put
    // "parallel to airframe" outside the standard 65-115 window.
    // VERIFIED BY HAND TEST: all four fin tabs can reach parallel
    // without binding before this change was made.
    // RESTORE TO 25° when re-enabling active control.
    static constexpr float MAX_SERVO_DEFLECT_DEG    = 45.0f;

    // Velocity below which we INHIBIT control (drag force too low for
    // meaningful fin authority). Prevents division by tiny numbers.
    static constexpr float MIN_CONTROL_VELOCITY_MPS = 0.1f; // TEMP: bench test only

    // Synthetic velocity used when bench-test mode is enabled.
    static constexpr float BENCH_TEST_VELOCITY_MPS  = 50.0f;

  private:
    IMU&            _imu;
    Barometer&      _baro;
    StateEstimator& _estimator;

    Servo _servoA, _servoB, _servoC, _servoD;
    int   _servoAPin, _servoBPin, _servoCPin, _servoDPin;

    int _trimA = 0, _trimB = 0, _trimC = 0, _trimD = 0;

    bool _controlEnabled = false;
    bool _benchTestMode  = false;

    uint32_t _lastUpdateMicros = 0;
    bool     _firstUpdate      = true;

    float _prevRollErr  = 0.0f;
    float _prevPitchErr = 0.0f;
    float _prevYawErr   = 0.0f;

    float _cmdA = 90.0f, _cmdB = 90.0f, _cmdC = 90.0f, _cmdD = 90.0f;
    float _lastRollErr   = 0.0f;
    float _lastPitchErr  = 0.0f;
    float _lastYawErr    = 0.0f;
    float _lastDragForce = 0.0f;

    float computeDragForce(float velocity_mps) const;
    void  writeServos(float cA, float cB, float cC, float cD);
    void  rotateImuRatesToAirframe(float pitchRate_imu, float yawRate_imu,
                                   float& pitchRate_af, float& yawRate_af) const;

    // --- Aerodynamic geometry of ONE fin tab (the control surface actuated
    // by one MEX-12 servo). These are NOT main fin numbers; the main fins
    // are passive and do not appear in the controller math.
    static constexpr float CTRL_SURFACE_CD       = 0.5f;
    static constexpr float CTRL_SURFACE_AREA_M2 = 0.00138f;  // 57.5 × 24 mm tab
    // Moment arms. Axial arm uses burn-averaged CG; tab is aft of CG in all states.
    static constexpr float YAWPIT_TORQUE_ARM_M = 0.405f;    // tab CP to sustainer CG, axial
    static constexpr float ROLL_TORQUE_ARM_M   = 0.067f;    // tab CP to rocket centerline, radial
    static constexpr float R_DRY_AIR           = 287.05f;
    static constexpr float MIN_DRAG_FORCE_N    = 0.5f;

    bool _benchSuspend = false;
};

#endif