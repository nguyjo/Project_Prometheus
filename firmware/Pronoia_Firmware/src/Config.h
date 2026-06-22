#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// =============================================================================
// PROMETHEUS AVIONICS — SYSTEM CONFIGURATION
// =============================================================================
// All compile-time configuration lives here. Change values, recompile, reflash.
// DO NOT scatter magic numbers across multiple files — put them here.
// =============================================================================

namespace Config {

namespace Build {

#if defined(BUILD_FOR_BOOSTER)
  static constexpr const char* STAGE_NAME = "BOOSTER";
  static constexpr bool IS_BOOSTER  = true;
  static constexpr bool IS_SUSTAINER = false;
#elif defined(BUILD_FOR_SUSTAINER)
  static constexpr const char* STAGE_NAME = "SUSTAINER";
  static constexpr bool IS_BOOSTER  = false;
  static constexpr bool IS_SUSTAINER = true;
#else
  #error "Must define BUILD_FOR_BOOSTER or BUILD_FOR_SUSTAINER in platformio.ini"
#endif

// Build timestamp — embedded automatically by the compiler at build time.
// Useful for verifying which firmware version is loaded when debugging.
static constexpr const char* BUILD_DATE = __DATE__;
static constexpr const char* BUILD_TIME = __TIME__;

}  // namespace Build

// ---------- Hardware Pin Assignments ----------
namespace Pins {
    constexpr int IMU_CS         = 10;
    constexpr int HIGHG_CS       = 37;
    constexpr int BARO_CS        = 0;
    // constexpr int FLASH_CS       =; Flash Chip communicates over Teensy 4.1 Integrated QPI, so no CS pin needed

    constexpr int EXTERNAL_CONN_DETECT = 18;   // GX12 detect pin

    constexpr int PYRO0_FIRE     = 41;
    constexpr int PYRO1_FIRE     = 40;
    constexpr int PYRO2_FIRE     = 39;
    constexpr int PYRO3_FIRE     = 38;
    constexpr int PYRO0_CONT     = 36;
    constexpr int PYRO1_CONT     = 35;
    constexpr int PYRO2_CONT     = 34;
    constexpr int PYRO3_CONT     = 33;

    // Buzzer pins
    constexpr int UI_BUZZER       = 5;
    constexpr int RECOVERY_BUZZER = 3;

    // ===== Canard servo pins (sustainer only) =====
    // Teensy 4.1 PWM-capable pins, one per MEX-12 servo.
    //
    // Physical geometry (per mechanical team):
    //   A is at Y+  
    //   C is at Y-   
    //   B is at Z-  
    //   D is at Z+ 
    //
    // Control implications:
    //   A and C lie on the Y-axis diameter, so their deflection produces
    //     a YAW couple (rotation about the Z body axis).
    //   B and D lie on the Z-axis diameter, so their deflection produces
    //     a PITCH couple (rotation about the Y body axis).
    //
    // This determines RateDampingController::AC_IS_PITCH_PAIR = false.
    // Verify with bench test (see RateDampingController.h doc-block).
    constexpr int CANARD_SERVO_A = 24;   // Y+
    constexpr int CANARD_SERVO_D = 25;   // Z+
    constexpr int CANARD_SERVO_C = 28;   // Y-
    constexpr int CANARD_SERVO_B = 29;   // Z-; B (Z-) is marked on sled

    // 2S LiPo voltage divider: 22kΩ / 10kΩ → divided by 3.2
    // Hardware low-pass filter (100nF cap) corners at ~230 Hz
    constexpr int VBATT_MONITOR_PIN = 14;
}  // namespace Pins

// ---------- Sensor Sampling Rates ----------
namespace Rates {
    constexpr uint32_t IMU_HZ       = 1000;
    constexpr uint32_t HIGHG_HZ     = 800;   // KX134 ODR setting
    constexpr uint32_t BARO_HZ      = 50;    // MS5611 with OSR=4096
    constexpr uint32_t CONTROL_HZ   = 100;   // Outer control loop (servos)

    // No Telemetry in 2025-2026 Prometheus Rocket Project
    // constexpr uint32_t TELEMETRY_HZ = 20;    // Serial4 to ground
}  // namespace Rates

namespace Control {
  // EMERGENCY 2-FIN MODE
  // Set to true to fly with only servos B and D.
  // Use when servos A and/or C are unavailable or unreliable.
  //
  // Authority profile in this mode:
  //   - Pitch authority: full (B + D anti-symmetric couple)
  //   - Yaw authority:   ZERO (no Y-axis fins — relies on passive stability)
  //   - Roll authority:  ZERO (commanded — see RateDampingController.cpp
  //                      for rationale; roll-pitch coupling on a Z-only
  //                      couple makes roll control unsafe)
  //
  // Servos A and C are held actively centered at every control cycle.
  // Trim values for A and C are still applied — see A3b procedure.
  constexpr bool TWO_FIN_MODE = true;
}

// ---------- Pyro Configuration ----------
namespace Pyro {
    constexpr uint32_t FIRE_TIME_MS = 500;   // Duration of fire pulse for ignition
    constexpr uint32_t ARM_TIMEOUT_MS = 5000;  // ARM→FIRE window

    // ===== Pyro channel assignments — VERIFY against physical wiring =====
    //
    // BOOSTER firmware fires:
    //   PYRO0 = booster main parachute charge (at 1,500 ft on descent)
    //   PYRO1, 2, 3 = unused, no ematches installed
    //
    // SUSTAINER firmware fires:
    //   PYRO0 = sustainer ignitor (mid-air relight after separation + tilt check)
    //   PYRO1 = sustainer main parachute charge (at 1,500 ft on descent)
    //   PYRO2, 3 = unused, no ematches installed
    //
    // Note: apogee deployment for BOTH stages is MECHANICAL (motor ejection
    // charge for the sustainer, separation-triggered drogue release for the
    // booster). Firmware does NOT fire any apogee pyros.
    constexpr int PYRO_BOOSTER_MAIN     = 0;
    constexpr int PYRO_SUSTAINER_IGNITE = 0;
    constexpr int PYRO_SUSTAINER_MAIN   = 1;
}  // namespace Pyro

// ---------- Flight Profile Limits & Safety ----------
namespace Safety {
    // ===== Liftoff detection (both stages) =====
    // Sustained axial accel above this for required duration latches liftoff.
    // 2.5g is well above noise (resting 1g) but well below any motor's spike.
    // For the sustainer, this triggers on BOOSTER ignition since they're
    // hot-staged — the sustainer is along for the ride during booster burn.
    constexpr float    LIFTOFF_ACCEL_MPS2  = 2.5f * 9.80665f;
    constexpr uint32_t LIFTOFF_DURATION_MS = 100;

    // ===== Burnout detection (both stages) =====
    // Sustained low accel = motor has burned out (we're in ballistic flight).
    // The threshold is "no thrust present" — anything below ~0.5g of axial accel
    // is consistent with coasting.
    constexpr float    BURNOUT_ACCEL_MPS2  = 0.5f * 9.80665f;
    constexpr uint32_t BURNOUT_DURATION_MS = 200;

    // ===== Main deploy altitude (both stages) =====
    // Standard for L2: 1,500 ft = ~457m. Verify with your recovery system.
    constexpr float MAIN_DEPLOY_ALT_M      = 457.0f;

    // ===== Touchdown detection (both stages) =====
    constexpr float    TOUCHDOWN_VEL_MAX_MPS = 2.0f;
    constexpr float    TOUCHDOWN_ALT_MAX_M   = 100.0f;
    constexpr uint32_t TOUCHDOWN_DURATION_MS = 5000;

    // ===== Sustainer-specific: separation delay and tilt check =====
    // After booster burnout, the sustainer firmware waits this long before
    // attempting to fire the sustainer ignitor. The wait allows:
    //   - Physical separation to occur
    //   - Booster drogue to deploy mechanically (~90 m/s)
    //   - Rocket to settle into stable trajectory for accurate tilt reading
    //
    // Per the flight profile: ~7 seconds of booster coast between burnout
    // and sustainer ignition.
    constexpr uint32_t SEPARATION_DELAY_MS = 7000;

    // After the separation delay, the firmware has this window to verify
    // tilt and ignite. If tilt fails OR the window expires, transition to
    // ABORT and skip sustainer ignition entirely.
    constexpr uint32_t SUSTAINER_IGN_WINDOW_MS = 500;

    // Maximum allowed tilt (degrees) for sustainer ignition.
    // Per team decision: 45° (more permissive than the originally-suggested 20°).
    //
    // NOTE: This value is duplicated in TiltEstimator.h as MAX_TILT_DEG.
    // Keep them in sync. If you change one, change the other.
    constexpr float MAX_SUSTAINER_TILT_DEG = 45.0f;
}  // namespace Safety

namespace Estimator {
    constexpr float ALPHA_VELOCITY            = 0.98f;
    constexpr float ALPHA_ALTITUDE            = 0.96f;
    constexpr float BARO_VEL_TRUST_MAX_MPS    = 300.0f;
    constexpr float BARO_ALT_TRUST_MAX_M      = 5000.0f;
    constexpr float COAST_THRESHOLD_MPS2      = 0.2f * 9.80665f;
}  // namespace Estimator

namespace Power {
    // Voltage divider math: V_battery = V_monitor × (R23 + R24) / R24
    constexpr float VBATT_DIVIDER_RATIO = (22.0f + 10.0f) / 10.0f;  // = 3.2
    
    // Teensy 4.1 ADC: 10-bit by default (0..1023), 3.3V reference.
    // We'll switch to 12-bit (0..4095) in setup() for better resolution.
    constexpr float ADC_REFERENCE_V    = 3.3f;
    constexpr int   ADC_MAX_COUNTS     = 4095;  // 12-bit
    
    // Health thresholds for 2S LiPo
    constexpr float VBATT_FULL_V       = 8.40f;   // freshly charged
    constexpr float VBATT_NOMINAL_V    = 7.40f;   // nominal mid-discharge
    constexpr float VBATT_WARN_V       = 7.20f;   // sag warning threshold
    constexpr float VBATT_CRITICAL_V   = 6.60f;   // stop testing
    constexpr float VBATT_LVC_V        = 6.00f;   // LiPo low-voltage cutoff
}

}  // namespace Config

#endif