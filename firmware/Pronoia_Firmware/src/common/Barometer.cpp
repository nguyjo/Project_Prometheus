#include "Barometer.h"

Barometer::Barometer(int csPin) {
  _csPin = csPin;
}

bool Barometer::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  // Send Software Reset
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI1.transfer(0x1E);
  digitalWrite(_csPin, HIGH);
  SPI1.endTransaction();

  delay(5); // Wait 5ms for the PROM to reload its memory into the registers

  // Extract the 6 Factory Calibration Variables
  C1 = readPROM(0xA2); // Pressure Sensitivity
  C2 = readPROM(0xA4); // Pressure Offset
  C3 = readPROM(0xA6); // Temperature Coefficient of Pressure Sensitivity
  C4 = readPROM(0xA8); // Temperature Coefficient of Pressure Offset
  C5 = readPROM(0xAA); // Reference Temperature
  C6 = readPROM(0xAC); // Temperature Coefficient of the Temperature

  // // Check 6 Coefficients read from PROM
  // Serial.print("Raw C1 from bus: "); 
  // Serial.println(C1);
  // Serial.print("Raw C2 from bus: "); 
  // Serial.println(C2);
  // Serial.print("Raw C3 from bus: "); 
  // Serial.println(C3);
  // Serial.print("Raw C4 from bus: "); 
  // Serial.println(C4);
  // Serial.print("Raw C5 from bus: ");
  // Serial.println(C5);
  // Serial.print("Raw C6 from bus: ");
  // Serial.println(C6);

  // If any is Coefficient is 0 or 65535, the PROM read failed
  if (C1 == 0x0000 || C1 == 0xFFFF || C2 == 0x0000 || C2 == 0xFFFF || C3 == 0x0000 || C3 == 0xFFFF ||
      C4 == 0x0000 || C4 == 0xFFFF || C5 == 0x0000 || C5 == 0xFFFF || C6 == 0x0000 || C6 == 0xFFFF) {
    return false;
  }
  
  return true;
}

// Helper function to read 16 bits from the PROM
uint16_t Barometer::readPROM(uint8_t cmd) {
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  
  SPI1.transfer(cmd); // Ask for the specific PROM address
  uint8_t highByte = SPI1.transfer(0x00);
  uint8_t lowByte = SPI1.transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
  SPI1.endTransaction();
  
  return (highByte << 8) | lowByte;
}

// --- The Non-Blocking State Machine that pulls D1/D2 values ---
bool Barometer::update() {
  switch (_state) {
    case 0: // IDLE: Ask for Raw Pressure (D1)
      SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(_csPin, LOW);
      SPI1.transfer(0x48); // 0x48 = Convert D1 with highest resolution (OSR=4096)
      digitalWrite(_csPin, HIGH);
      SPI1.endTransaction();
      
      _lastRequestTime = micros(); // Record the exact microsecond we asked
      _state = 1; // Move to next state
      return false; // D1 data not ready yet

    case 1: // WAITING FOR D1
      // Highest resolution takes 9.04ms. We wait 9.5ms (9500 microseconds) to be safe.
      if (micros() - _lastRequestTime > 9500) {
        _D1 = readADC(); // Pick up the data
        
        // Immediately ask for Raw Temperature (D2)
        SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(_csPin, LOW);
        SPI1.transfer(0x58); // 0x58 = Convert D2 (OSR=4096)
        digitalWrite(_csPin, HIGH);
        SPI1.endTransaction();
        
        _lastRequestTime = micros();
        _state = 2; 
      }
      return false; // D2 data not ready yet

    case 2: // WAITING FOR D2
      if (micros() - _lastRequestTime > 9500) {
        _D2 = readADC(); // Read D2
        calculateMath(); // Calculate altitude_AGL
        updateDerivedSignals(); // compute smoothed altitude and velocity
        _state = 0;      // Loop back to the beginning
        return true;     // New altitude_AGL data is ready!
      }
      return false;
  }
  return false;
}

// Extract the 24-bit raw D1/D2 values from the ADC
uint32_t Barometer::readADC() {
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI1.transfer(0x00); // Send ADC Read Command
  
  uint8_t b1 = SPI1.transfer(0x00); // Top 8 bits
  uint8_t b2 = SPI1.transfer(0x00); // Middle 8 bits
  uint8_t b3 = SPI1.transfer(0x00); // Bottom 8 bits
  
  digitalWrite(_csPin, HIGH);
  SPI1.endTransaction();
  
  return ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
}

// Calculating Temperature, temperature-compensated pressure, and altitude using the MS5611's recommended math from the datasheet
void Barometer::calculateMath() {
  // --- FIRST ORDER MATH ---
  int32_t dT = _D2 - ((int32_t)C5 << 8);
  int32_t TEMP = 2000 + (((int64_t)dT * C6) >> 23);
  
  int64_t OFF = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);
  int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);
  
  // --- SECOND ORDER TEMPERATURE COMPENSATION ---
  int64_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  if (TEMP < 2000) { // If temperature is below 20.00 C
    T2 = ((int64_t)dT * dT) >> 31;
    
    int64_t tempDiff = TEMP - 2000;
    int64_t sqDiff = tempDiff * tempDiff;
    
    OFF2 = (5 * sqDiff) >> 1;
    SENS2 = (5 * sqDiff) >> 2;

    if (TEMP < -1500) { // If temperature is below -15.00 C
      int64_t deepColdDiff = TEMP + 1500;
      int64_t deepColdSq = deepColdDiff * deepColdDiff;
      
      OFF2 = OFF2 + (7 * deepColdSq);
      SENS2 = SENS2 + ((11 * deepColdSq) >> 1);
    }
  }

  // Apply the second-order corrections
  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
  // ----------------------------------------------

  // Final Pressure Calculation
  int32_t P = (((_D1 * SENS) >> 21) - OFF) >> 15;

  // Convert to human-readable floats
  temp_C = TEMP / 100.0;
  pressure_Mbar = P / 100.0;
  
  // Standard barometric altitude formula, calculating altitude relative to sea level.
  altitude_MSL = 44330.76923 * (1.0 - pow((pressure_Mbar / 1013.25), 0.1902632365));

  // Calculate AGL by subtracting the ground reference altitude (set at liftoff) from the current MSL altitude.
  altitude_AGL = altitude_MSL - groundAltitude_MSL;
}

// Calibrate the barometer by averaging the current altitude over a number of samples to set the groundAltitude_MSL reference. Call this at liftoff!
bool Barometer::calibrateBaro(int samples) {
  // Flush first sample (initial state-machine sync)
  unsigned long flushStart = millis();
  while (!update()) {
    if (millis() - flushStart > 100) return false;  // Timeout if update() doesn't return true within 100ms, likely a sensor issue. We want to fail fast here.
  }

  float sumMSL = 0.0f;
  int validSamples = 0;
  unsigned long calStart = millis(); // Calibration start time
  const unsigned long maxCalTime = (unsigned long)samples * 30UL + 500UL; // Conservative Maximum Cal Time > 20 mx * update()

  while (validSamples < samples) {
    if (update()) {
      sumMSL += altitude_MSL;
      validSamples++;
    }
    if (millis() - calStart > maxCalTime) return false;  // Sensor dead
  }

  groundAltitude_MSL = sumMSL / samples;
  return true;
}

// ============================================================
// updateDerivedSignals
// ============================================================
// Called once per fresh barometer sample (~50 Hz). Maintains:
//   1. A 10-sample moving average of altitude_AGL → smoothedAltitude_AGL
//   2. A 10-sample-lagged finite-difference velocity → velocity_mps
//
// The lagged differentiation is critical: differentiating consecutive
// samples gives ±25 m/s of noise from ±0.5m altitude noise at 20ms
// sampling. Using a 10-sample baseline (~200ms) reduces this to ~2.5 m/s
// at the cost of 200ms phase lag — acceptable for telemetry and apogee
// detection, NOT acceptable for a real-time control loop.
// ============================================================
void Barometer::updateDerivedSignals() {
  // --- Step 1: Maintain smoothed altitude (10-sample moving average) ---
  _smoothAltSum -= _smoothAltBuffer[_smoothAltBufferPosition];
  _smoothAltBuffer[_smoothAltBufferPosition] = altitude_AGL;
  _smoothAltSum += altitude_AGL;
  smoothedAltitude_AGL = _smoothAltSum / (float)SMOOTH_BUFFER_SIZE;

  _smoothAltBufferPosition++;
  if (_smoothAltBufferPosition >= SMOOTH_BUFFER_SIZE) {
    _smoothAltBufferPosition = 0;
  }

  // Track peak altitude for apogee detection downstream
  if (smoothedAltitude_AGL > maxAltitude_AGL) {
    maxAltitude_AGL = smoothedAltitude_AGL;
  }

  // --- Step 2: Maintain altitude history buffer for lagged differentiation ---
  unsigned long nowMicros = micros();
  _velAltBuffer[_velBufferPosition]  = smoothedAltitude_AGL;
  _velTimeBuffer[_velBufferPosition] = nowMicros;

  // --- Step 3: Compute velocity over 10-sample lag window ---
  if (_velBufferFilled) {
    int laggedPosition = (int)_velBufferPosition - (int)VEL_LAG_SAMPLES;
    if (laggedPosition < 0) {
      laggedPosition += VEL_BUFFER_SIZE;
    }

    float deltaAlt = smoothedAltitude_AGL - _velAltBuffer[laggedPosition];
    float deltaTimeSec = (float)(nowMicros - _velTimeBuffer[laggedPosition]) * 1.0e-6f;

    if (deltaTimeSec > 0.001f) {
      velocity_mps = deltaAlt / deltaTimeSec;
    }

    if (velocity_mps > maxVelocity_mps) {
      maxVelocity_mps = velocity_mps;
    }
  }

  // Advance position pointer with wraparound
  _velBufferPosition++;
  if (_velBufferPosition >= VEL_BUFFER_SIZE) {
    _velBufferPosition = 0;
    _velBufferFilled = true;
  }
}