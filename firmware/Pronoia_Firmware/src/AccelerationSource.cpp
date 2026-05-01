#include "AccelerationSource.h"
#include <math.h>

AccelerationSource::AccelerationSource(IMU& imu, HighGAccel& highG)
  : _imu(imu), _highG(highG) {}

void AccelerationSource::update() {
  // Pull rocket-frame axial readings from each sensor.
  // The accessors handle chip-to-rocket axis remapping internally.
  _rawIMUAccel_g   = _imu.axialAccelG();
  _rawHighGAccel_g = _highG.axialAccelG();

  // Decide which sensor to trust based on IMU saturation.
  if (fabsf(_rawIMUAccel_g) < IMU_SATURATION_THRESHOLD_G) {
    // IMU is in valid range — use it (lower noise, better precision near 1g).
    _usingHighG = false;
    _axialAccel_mps2 = (_rawIMUAccel_g - 1.0f) * G_TO_MPS2;
  } else {
    // IMU saturated — fall back to KX134.
    _usingHighG = true;
    _axialAccel_mps2 = (_rawHighGAccel_g - 1.0f) * G_TO_MPS2;
  }
}