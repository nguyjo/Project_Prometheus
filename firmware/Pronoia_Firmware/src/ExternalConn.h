#ifndef EXTERNALCONN_H
#define EXTERNALCONN_H

#include <Arduino.h>
#include <IMU.h>
#include <HighGAccel.h>
#include <AccelerationSource.h>

class ExternalConn {
  public:
    // Constructor takes references to all sensors needed for diagnostics
    ExternalConn(int detectPin, IMU& imu, HighGAccel& highG, AccelerationSource& accelSrc);


    // Methods
    void begin();
    void update();
    bool isConnected();

    // Flight Flags (Public so main.cpp can read them)
    bool isArmed;
    bool isLogging;

  private:
    int _detectPin;

    IMU& _imu;
    HighGAccel& _highG;
    AccelerationSource& _accelSrc;

    // Streaming state
    bool _streaming = false;
    unsigned long _lastStreamPrintMs = 0;
    static constexpr unsigned long STREAM_INTERVAL_MS = 200;  // 20 Hz

    void processCommand(String cmd);
    void streamSensorReadings();
};

#endif