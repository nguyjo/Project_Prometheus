
// =============================================================================
// BenchTool
// =============================================================================
// Bench-side diagnostic interface, listening on USB Serial (NOT Serial4).
//
// This interface is intended for development, debugging, and bench testing.
// It exposes raw sensor access and other low-level diagnostic features that
// are intentionally NOT available via the pad-time ExternalConn interface.
//
// At the launch site, no USB cable is connected, so BenchTool commands are
// physically unreachable. This is defense-in-depth against accidental
// invocation of debug commands during flight operations.
// =============================================================================

#ifndef BENCHTOOL_H
#define BENCHTOOL_H

#include <Arduino.h>
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "Barometer.h"

class BenchTool {
  public:
    BenchTool(IMU& imu, HighGAccel& highG, AccelerationSource& accelSrc, Barometer& baro);

    void begin();
    void update();

  private:
    IMU& _imu;
    HighGAccel& _highG;
    AccelerationSource& _accelSrc;
    Barometer& _baro;

    bool _streaming = false;
    unsigned long _lastStreamPrintMs = 0;
    static constexpr unsigned long STREAM_INTERVAL_MS = 50;  // 20 Hz

    void processCommand(String cmd);
    void streamSensorReadings();
    void printHelp();
};

#endif