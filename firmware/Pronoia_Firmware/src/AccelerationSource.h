#ifndef ACCELERATIONSOURCE_H
#define ACCELERATIONSOURCE_H

#include "IMU.h"
#include "HighGAccel.h"

// =============================================================================
// AccelerationSource
// =============================================================================
// Adapter class that presents a single, gravity-removed, rocket-frame axial
// acceleration value to the rest of the flight software. Internally, it
// switches between the primary IMU (low noise, ±16g) and the KX134 high-G
// accelerometer (±64g) based on IMU saturation.
//
// USAGE:
//   1. Construct with references to existing IMU and HighGAccel objects.
//   2. Each main loop iteration, call imu.update() and highG.update() FIRST,
//      then call accelSource.update().
//   3. Read getAxialAccelMps2() to get the value to feed to the StateEstimator.
// =============================================================================

class AccelerationSource {
  public:
    AccelerationSource(IMU& imu, HighGAccel& highG);

    // Call once per main loop iteration AFTER both sensors have been update()'d
    void update();

    // Axial acceleration along rocket Z-axis (gravity removed), in m/s².
    // This is the value to feed into velocity/altitude integration.
    float getAxialAccelMps2() const { return _axialAccel_mps2; }

    // Diagnostics & telemetry
    bool isUsingHighG() const       { return _usingHighG; }
    float getRawIMUAxialG() const   { return _rawIMUAccel_g; }
    float getRawHighGAxialG() const { return _rawHighGAccel_g; }

  private:
    IMU& _imu;
    HighGAccel& _highG;

    float _axialAccel_mps2  = 0.0f;
    float _rawIMUAccel_g    = 0.0f;
    float _rawHighGAccel_g  = 0.0f;
    bool  _usingHighG       = false;

    // Saturation threshold — switch to KX134 when IMU exceeds this in g.
    // Set below ±16g limit to switch BEFORE clipping artifacts appear.
    static constexpr float IMU_SATURATION_THRESHOLD_G = 15.5f;
    static constexpr float G_TO_MPS2 = 9.80665f;
};

#endif