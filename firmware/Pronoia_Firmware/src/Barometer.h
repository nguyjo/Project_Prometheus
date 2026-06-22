#ifndef Barometer_H
#define Barometer_H

#include <Arduino.h>
#include <SPI.h>

class Barometer {
  public:
    Barometer(int csPin);
    bool begin();
    bool update();  // script that reads current D1/D2 values
                    // The script's non-blocking nature allows teensy to run other commands while waiting for ADC conversions to complete.

    bool calibrateBaro(int samples); // Averages the current altitude over a number of samples to set the groundAltitude_MSL reference. Call this at liftoff!
    
    // ===== NEW: Sample-availability flag =====
    // Returns true if a fresh sample has been produced since the last call.
    // Calling this method CONSUMES the flag (resets it to false). Callers that
    // want to inspect freshness without consuming should not use this — there
    // is intentionally only the consuming accessor to prevent the common bug
    // of multiple consumers each thinking they own the sample.
    bool consumeNewSampleFlag();

    // We will store the factory calibration variables here
    uint16_t C1, C2, C3, C4, C5, C6;

    // Human-readable flight data
    float temp_C = 0.0f;              // Temperature in degrees Celsius
    float pressure_Mbar = 0.0f;       // Pressure in millibars (hPa)
    float altitude_MSL = 0.0f;        // Altitude above Mean Sea Level
    float groundAltitude_MSL = 0.0f;  // Set by calibrateBaro() before flight; Launchpad reference altitude above Mean Sea Level, set at liftoff
    float altitude_AGL= 0.0f;         // Live flight altitude above ground level

    // ============================================================
    // SMOOTHED & VELOCITY-DERIVED SIGNALS (added for sensor fusion)
    // ============================================================
    // smoothedAltitude_AGL: low-pass-filtered altitude (10-sample moving avg)
    //                       reduces ±0.5m sensor noise to ~±0.16m.
    // velocity_mps:         finite-difference velocity over 10-sample lag window
    //                       (~200ms baseline at 50Hz baro rate). Trades 200ms
    //                       phase lag for dramatically reduced noise.
    // maxVelocity_mps:      tracked for sanity gating during sensor fusion.
    // maxAltitude_AGL:      tracked for apogee detection downstream.
    float smoothedAltitude_AGL = 0.0f;
    float velocity_mps         = 0.0f;
    float maxVelocity_mps      = 0.0f;
    float maxAltitude_AGL      = 0.0f;
  private:
    int _csPin;
    uint16_t readPROM(uint8_t cmd);

    uint32_t readADC(); // Reading D1 and D2 values from the ADC
    void calculateMath(); // Calculate temperature, and temperature compensated pressure, and altitude

    // Maintains smoothed altitude and lagged-difference velocity.
    // Called from update() when a fresh barometer sample is ready.
    void updateDerivedSignals();

    int _state = 0; 
    unsigned long _lastRequestTime = 0;
    uint32_t _D1 = 0; // Raw Pressure
    uint32_t _D2 = 0; // Raw Temp

    // ============================================================
    // SMOOTHING & VELOCITY BUFFERS
    // ============================================================
    static constexpr uint8_t SMOOTH_BUFFER_SIZE = 10;
    static constexpr uint8_t VEL_BUFFER_SIZE    = 30;
    static constexpr uint8_t VEL_LAG_SAMPLES    = 10;

    float _smoothAltBuffer[SMOOTH_BUFFER_SIZE] = {0}; // array of 10 0's for moving average
    float _smoothAltSum = 0.0f;
    uint8_t _smoothAltBufferPosition = 0;

    float         _velAltBuffer[VEL_BUFFER_SIZE]  = {0};
    unsigned long _velTimeBuffer[VEL_BUFFER_SIZE] = {0};
    uint8_t _velBufferPosition = 0;
    bool _velBufferFilled = false;

    // ===== NEW: Sample-availability tracking =====
    // Set to true inside update() when calculateMath() + updateDerivedSignals()
    // have completed and a fresh sample is available. Consumers call
    // consumeNewSampleFlag() to read-and-clear.
    bool _newSampleReady = false;
};

#endif

