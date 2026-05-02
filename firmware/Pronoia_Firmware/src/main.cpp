/*
ExternalConn commands (bench mode, manual):
  STREAM_SENSORS    // Continuous data for axis verification
  CAL_GYRO          // Manual gyro calibration trigger (also useful in lab)
  CAL_BARO          // Manual baro reference set
  PYRO_TEST         // Manual pyro continuity check

ExternalConn commands (pad mode, semi-automated):
  PREFLIGHT         // Runs the full automated sequence:
                    //   1. cal_gyro (3 sec)
                    //   2. cal_baro (3 sec)
                    //   3. read initial accel, compute tilt
                    //   4. check tilt < 10°
                    //   5. check pyro continuity
                    //   6. check battery voltage
                    //   7. report PASS/FAIL with details
  ARM               // Allowed only if PREFLIGHT passed


  src/
  common/                          ← shared by both stages
    IMU.{h,cpp}
    HighGAccel.{h,cpp}
    Barometer.{h,cpp}
    AccelerationSource.{h,cpp}
    StateEstimator.{h,cpp}
    PyroChannel.{h,cpp}
    ExternalConn.{h,cpp}
    BenchTool.{h,cpp}
    Config.h
  flight/
    FlightStrategy.h               ← abstract interface
    BoosterFlightStrategy.{h,cpp}  ← booster-specific
    SustainerFlightStrategy.{h,cpp}← sustainer-specific
    OrientationInterlock.{h,cpp}   ← only used by sustainer
    RateDampingController.{h,cpp}  ← only used by sustainer
  main.cpp                         ← single entry point with #ifdef selector
*/

#include <Arduino.h>
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "BenchTool.h"
#include "ExternalConn.h"
#include "Config.h"
// #include "BoosterFlightStrategy.h"
// #include "SustainerFlightStrategy.h"

// =============================================================================
// STAGE SELECTION — change this define to build for booster vs sustainer
// =============================================================================
// FlightStrategy is scaffolded but not yet implemented. For now, stage selection
// only affects the Config::Build constants and the startup banner. When stage-
// specific flight logic is added, instantiate the appropriate strategy here.

// #define BUILD_FOR_SUSTAINER   // or BUILD_FOR_BOOSTER

// #if defined(BUILD_FOR_BOOSTER)
//   BoosterFlightStrategy flightStrategy(/* deps */);
//   static constexpr const char* BUILD_TARGET = "BOOSTER";
// #elif defined(BUILD_FOR_SUSTAINER)
//   SustainerFlightStrategy flightStrategy(/* deps */);
//   static constexpr const char* BUILD_TARGET = "SUSTAINER";
// #else
//   #error "Must define BUILD_FOR_BOOSTER or BUILD_FOR_SUSTAINER"
// #endif

// Sensor objects — fill in your actual CS pins
IMU         imu(Config::Pins::IMU_CS);
HighGAccel  highG(Config::Pins::HIGHG_CS);
AccelerationSource accelSrc(imu, highG);
Barometer baro(Config::Pins::BARO_CS);

// Pad-time interface (GX12 umbilical via Serial4)
ExternalConn externalConn(Config::Pins::EXTERNAL_CONN_DETECT);

// Bench-time interface (USB via Serial)
BenchTool benchTool(imu, highG, accelSrc, baro);

void setup() {
  SPI.begin();   // SPI bus 0 — for IMU and High-G accel; ICM-42688-PC, KX134-1211
  SPI1.begin();  // SPI bus 1 — for Barometer; MS5611-01BA03
  Serial.begin(115200);
  delay(500);

  // Print build identification banner — appears on USB serial at boot
  Serial.println();
  Serial.println("==========================================");
  Serial.printf("  PROMETHEUS AVIONICS — %s STAGE\n", Config::Build::STAGE_NAME);
  Serial.printf("  Build: %s %s\n", Config::Build::BUILD_DATE, Config::Build::BUILD_TIME);
  Serial.println("==========================================");
  Serial.println();

  externalConn.begin();
  benchTool.begin();

  // Initialize sensors
  if (!imu.begin()) {
    Serial.println("IMU init FAILED");
    Serial4.println("IMU init FAILED");
  }
  if (!highG.begin()) {
    Serial.println("HighG init FAILED");
    Serial4.println("HighG init FAILED");
  }
  if (!baro.begin()) {
    Serial.println("Baro init FAILED");
    Serial4.println("Baro init FAILED");
  }
}

void loop() {
  // Update sensors first (order matters!)
  imu.update();
  highG.update();
  baro.update();
  accelSrc.update();

  externalConn.update();   // Listens on Serial4 (umbilical)
  benchTool.update();      // Listens on Serial (USB)

  // Flight logic placeholder
  if (externalConn.isConnected()) {
    // GROUND MODE — listen for commands
  } else {
    // FLIGHT MODE
    if (externalConn.isArmed) {
      // Run sensor fusion, apogee detection, etc.
    }
  }
}