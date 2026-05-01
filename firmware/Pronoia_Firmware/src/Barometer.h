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
    
    // We will store the factory calibration variables here
    uint16_t C1, C2, C3, C4, C5, C6;

    // Human-readable flight data
    float temp_C;             // Temperature in degrees Celsius
    float pressure_Mbar;      // Pressure in millibars (hPa)
    float altitude_MSL;       // Altitude above Mean Sea Level
    float groundAltitude_MSL; // Launchpad reference altitude above Mean Sea Level, set at liftoff
    float altitude_AGL;       // Live flight altitude above ground level

  private:
    int _csPin;
    uint16_t readPROM(uint8_t cmd);

    uint32_t readADC(); // Reading D1 and D2 values from the ADC
    void calculateMath(); // Calculate temperature, and temperature compensated pressure, and altitude

    int _state = 0; 
    unsigned long _lastRequestTime = 0;
    uint32_t _D1 = 0; // Raw Pressure
    uint32_t _D2 = 0; // Raw Temp
};

#endif

