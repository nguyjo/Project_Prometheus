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
*/

#include <Arduino.h>
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "ExternalConn.h"
#include "Config.h"

// Sensor objects — fill in your actual CS pins
IMU         imu(Config::Pins::IMU_CS);
HighGAccel  highG(Config::Pins::HIGHG_CS);
AccelerationSource accelSrc(imu, highG);

// External connection now takes references to all three
ExternalConn externalConn(Config::Pins::EXTERNAL_CONN_DETECT, imu, highG, accelSrc);

void setup() {
  Serial.begin(115200);
  externalConn.begin();

  // Initialize sensors
  if (!imu.begin())      { Serial4.println("IMU init FAILED"); }
  if (!highG.begin())    { Serial4.println("HighG init FAILED"); }
}

void loop() {
  // Update sensors first (order matters!)
  imu.update();
  highG.update();
  accelSrc.update();

  // Service the external connection (handles streaming + commands)
  externalConn.update();

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