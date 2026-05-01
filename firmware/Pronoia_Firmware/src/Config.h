#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// PROMETHEUS AVIONICS — SYSTEM CONFIGURATION
// =============================================================================
// All compile-time configuration lives here. Change values, recompile, reflash.
// DO NOT scatter magic numbers across multiple files — put them here.
// =============================================================================

namespace Config {

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
}

// ---------- Sensor Sampling Rates ----------
namespace Rates {
    constexpr uint32_t IMU_HZ       = 1000;
    constexpr uint32_t HIGHG_HZ     = 800;   // KX134 ODR setting
    constexpr uint32_t BARO_HZ      = 50;    // MS5611 with OSR=4096
    constexpr uint32_t CONTROL_HZ   = 100;   // Outer control loop (servos)

    // No Telemetry in 2025-2026 Prometheus Rocket Project
    // constexpr uint32_t TELEMETRY_HZ = 20;    // Serial4 to ground
}

// // ---------- Flight Profile Limits & Safety ----------
// namespace Safety {
//     // Sustainer ignition interlock — max off-vertical angle (degrees)
//     constexpr float MAX_SUSTAINER_TILT_DEG  = 20.0f;

//     // Booster separation timing
//     constexpr uint32_t SEPARATION_DELAY_MS  = /* fill in */;
//     constexpr uint32_t SUSTAINER_DELAY_MS   = /* fill in */;

//     // Pyro firing duration
//     constexpr uint32_t PYRO_FIRE_TIME_MS    = 500;

//     // Liftoff detection — sustained accel above this triggers liftoff
//     constexpr float LIFTOFF_ACCEL_G         = 2.5f;
//     constexpr uint32_t LIFTOFF_DURATION_MS  = 100;
// }

// // ---------- State Estimator Tuning ----------
// namespace Estimator {
//     constexpr float VEL_FUSION_TAU_S        = 1.0f;
//     constexpr float ALT_FUSION_TAU_S        = 0.2f;
//     constexpr float MAX_VALID_BARO_VEL_MPS  = 300.0f;
//     constexpr float MAX_VALID_BARO_ALT_M    = 13000.0f;
//     constexpr float COAST_THRESHOLD_MPS2    = 0.2f * 9.80665f;
// }

}  // namespace Config

#endif