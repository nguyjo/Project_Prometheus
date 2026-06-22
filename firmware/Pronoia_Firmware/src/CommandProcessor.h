// =============================================================================
// CommandProcessor.h
// =============================================================================
// Shared command-handling core. Receives a command string + the Stream that
// sent it, parses, executes, and writes responses back to that Stream.
//
// All commands work identically regardless of which Stream sent them. The
// detect-pin gating on the umbilical CommandLink is the only thing that
// physically distinguishes "umbilical" from "USB" at runtime.
//
// USAGE:
//   - CommandLink instances call process() with their Stream when they
//     have a complete line of input.
//   - main loop calls update() each iteration for housekeeping.
//   - main loop calls serviceStreaming(stream) from each CommandLink so
//     active sensor streaming gets serviced regardless of which interface
//     is sending.
// =============================================================================

#ifndef COMMANDPROCESSOR_H
#define COMMANDPROCESSOR_H

#include <Arduino.h>
#include "PyroChannel.h"
#include "Buzzer.h"
#include "FlashLogger.h"

// Forward declarations to keep this header light.
class IMU;
class HighGAccel;
class AccelerationSource;
class Barometer;
class RateDampingController;
class BatteryMonitor;
class TiltEstimator;

class CommandProcessor {
  public:
    CommandProcessor(
        PyroChannel& pyro0, PyroChannel& pyro1,
        PyroChannel& pyro2, PyroChannel& pyro3,
        Buzzer& uiBuzzer, Buzzer& recoveryBuzzer,
        IMU& imu, HighGAccel& highG,
        AccelerationSource& accelSrc, Barometer& baro,
        FlashLogger& flashLogger,
        BatteryMonitor& batteryMonitor,
        RateDampingController* rateDamper = nullptr,
        TiltEstimator* tiltEstimator = nullptr);

    // Process a single command string, writing all output to `out`.
    void process(const String& cmd, Stream& out);

    // Periodic housekeeping — call once per main loop iteration.
    void update();

    // Writes one streaming line to `out` if streaming is active AND `out` is
    // the Stream that owns the current streaming session. Called from each
    // CommandLink::update() so streaming output goes to whichever interface
    // started it, even when other interfaces are also being serviced.
    void serviceStreaming(Stream& out);

    // Flight-mode state accessors (read by main loop for flight-state gating).
    bool isFlightArmed() const { return _flightArmed; }
    bool isLogging()     const { return _logging; }

  private:
    // ----- Subsystem references -----
    PyroChannel& _pyro0;
    PyroChannel& _pyro1;
    PyroChannel& _pyro2;
    PyroChannel& _pyro3;
    PyroChannel* _pyros[4];

    Buzzer& _uiBuzzer;
    Buzzer& _recoveryBuzzer;

    IMU& _imu;
    HighGAccel& _highG;
    AccelerationSource& _accelSrc;
    Barometer& _baro;

    BatteryMonitor& _batteryMonitor;

    FlashLogger& _flashLogger;

    // ----- Two-step pyro arming state -----
    int8_t   _armedPyroIndex = -1;
    uint32_t _armedPyroAtMs  = 0;

    // ----- Flight-mode state -----
    bool _flightArmed = false;
    bool _logging     = false;

    // ----- Sensor streaming state -----
    bool          _streaming         = false;
    Stream*       _streamingTarget   = nullptr;
    unsigned long _lastStreamPrintMs = 0;
    static constexpr unsigned long STREAM_INTERVAL_MS = 50;  // 20 Hz

    // ----- Internal helpers -----
    void printHelp(Stream& out);
    void printStatus(Stream& out);
    void printReadyCheck(Stream& out);

    bool   anyChannelHasContinuity() const;
    int8_t parsePyroIndex(const String& cmd, const String& prefix) const;
    void   disarmPyro(Stream& out, bool quiet);
    void   emergencyStop(Stream& out);
    void   streamSensors(Stream& out);

    RateDampingController* _rateDamper = nullptr;

    TiltEstimator* _tiltEst = nullptr;
    // Tilt streaming state
    bool          _tiltStreaming         = false;
    Stream*       _tiltStreamingTarget   = nullptr;
    unsigned long _lastTiltStreamMs      = 0;
    static constexpr unsigned long TILT_STREAM_INTERVAL_MS = 200;  // 5 Hz
};

#endif