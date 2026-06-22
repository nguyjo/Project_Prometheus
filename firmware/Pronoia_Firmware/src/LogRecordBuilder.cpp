// =============================================================================
// LogRecordBuilder.cpp
// =============================================================================
#include "LogRecordBuilder.h"
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "StateEstimator.h"
#include "TiltEstimator.h"
#include "PyroChannel.h"

LogRecordBuilder::LogRecordBuilder(
    IMU& imu,
    HighGAccel& highG,
    AccelerationSource& accelSrc,
    Barometer& baro,
    StateEstimator& estimator,
    TiltEstimator* tiltEst,
    PyroChannel& pyro0, PyroChannel& pyro1,
    PyroChannel& pyro2, PyroChannel& pyro3)
  : _imu(imu), _highG(highG), _accelSrc(accelSrc), _baro(baro),
    _estimator(estimator), _tiltEst(tiltEst),
    _pyro0(pyro0), _pyro1(pyro1), _pyro2(pyro2), _pyro3(pyro3) {
}

LogRecord LogRecordBuilder::build(uint8_t flight_state,
                                  bool flight_armed,
                                  bool baro_calibrated,
                                  int8_t canard1, int8_t canard2,
                                  int8_t canard3, int8_t canard4) const {
  LogRecord rec = {};

  // ----- Timing and state -----
  rec.timestamp_us = micros();
  rec.flight_state = flight_state;

  // Pack the four pyro states into one byte: 2 bits per channel.
  // 0 = idle, 1 = armed, 2 = firing, 3 = spent (set externally if needed)
  auto encodePyroState = [](const PyroChannel& p) -> uint8_t {
    if (p.isFiring()) return 2;
    if (p.isArmed())  return 1;
    return 0;
  };
  rec.pyro_states = (encodePyroState(_pyro0) << 0)
                  | (encodePyroState(_pyro1) << 2)
                  | (encodePyroState(_pyro2) << 4)
                  | (encodePyroState(_pyro3) << 6);

  // Pack continuity into one byte, one bit per channel
  rec.pyro_continuity = (_pyro0.hasContinuity() ? 0x01 : 0)
                      | (_pyro1.hasContinuity() ? 0x02 : 0)
                      | (_pyro2.hasContinuity() ? 0x04 : 0)
                      | (_pyro3.hasContinuity() ? 0x08 : 0);

  // Pack flags
  rec.flags = 0;
  if (_tiltEst && _tiltEst->rotationFault()) rec.flags |= 0x01;
  if (_accelSrc.isUsingHighG())              rec.flags |= 0x02;
  if (flight_armed)                          rec.flags |= 0x04;
  if (baro_calibrated)                       rec.flags |= 0x08;

  // ----- Primary flight state -----
  rec.altitude_m       = _estimator.altitude_m();
  rec.velocity_mps     = _estimator.velocity_mps();
  rec.accel_axial_mps2 = _accelSrc.getAxialAccelMps2();
  rec.tilt_deg         = _tiltEst ? _tiltEst->tiltDeg() : 0.0f;

  // ----- Rotation rates -----
  rec.gyro_x_dps = _imu.pitchRateDps();
  rec.gyro_y_dps = _imu.rollRateDps();
  rec.gyro_z_dps = _imu.yawRateDps();

  // ----- Raw sensor readings -----
  rec.baro_altitude_m     = _baro.altitude_AGL;
  rec.imu_accel_g         = _imu.axialAccelG();
  rec.highg_accel_g       = _highG.axialAccelG();
  rec.baro_pressure_mbar  = _baro.pressure_Mbar;

  // ----- Lateral accelerations -----
  rec.imu_lat1_g = _imu.lat1AccelG();
  rec.imu_lat2_g = _imu.lat2AccelG();

  // ----- Canards (sustainer only — zero for booster) -----
  rec.canard1_deg = canard1;
  rec.canard2_deg = canard2;
  rec.canard3_deg = canard3;
  rec.canard4_deg = canard4;

  return rec;
}