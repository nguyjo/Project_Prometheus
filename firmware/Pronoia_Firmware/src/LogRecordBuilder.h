// =============================================================================
// LogRecordBuilder.h
// =============================================================================
// Helper that constructs a LogRecord from the current state of all subsystems.
//
// WHY THIS CLASS:
//   The FlashLogger doesn't care WHERE the data comes from — it just writes
//   the bytes you give it. The "where does the data come from" question is
//   answered here. Centralizing it makes it easy to:
//     - Add or remove fields without touching FlashLogger
//     - Verify all fields are populated correctly in one place
//     - Reuse for telemetry, debug printing, or other "dump current state"
//       operations
// =============================================================================

#ifndef LOGRECORDBUILDER_H
#define LOGRECORDBUILDER_H

#include "FlashLogger.h"

class IMU;
class HighGAccel;
class AccelerationSource;
class Barometer;
class StateEstimator;
class TiltEstimator;
class PyroChannel;

class LogRecordBuilder {
  public:
    LogRecordBuilder(
        IMU& imu,
        HighGAccel& highG,
        AccelerationSource& accelSrc,
        Barometer& baro,
        StateEstimator& estimator,
        TiltEstimator* tiltEst,           // nullable — booster doesn't have one
        PyroChannel& pyro0, PyroChannel& pyro1,
        PyroChannel& pyro2, PyroChannel& pyro3);

    // Construct a LogRecord from current sensor state.
    // flight_state is passed in because LogRecordBuilder doesn't have a
    // reference to the flight strategy (which would create circular deps).
    LogRecord build(uint8_t flight_state,
                    bool flight_armed,
                    bool baro_calibrated,
                    int8_t canard1 = 0, int8_t canard2 = 0,
                    int8_t canard3 = 0, int8_t canard4 = 0) const;

  private:
    IMU& _imu;
    HighGAccel& _highG;
    AccelerationSource& _accelSrc;
    Barometer& _baro;
    StateEstimator& _estimator;
    TiltEstimator* _tiltEst;
    PyroChannel& _pyro0;
    PyroChannel& _pyro1;
    PyroChannel& _pyro2;
    PyroChannel& _pyro3;
};

#endif