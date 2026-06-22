// =============================================================================
// BatteryMonitor.h
// =============================================================================
// Reads the 2S LiPo voltage via the resistive divider on the PCBA.
// Provides instantaneous readings plus a rolling min/avg/max over a window.
//
// The hardware divider (22k/10k) and 100nF cap give a clean DC reading with
// a ~230 Hz corner — slow enough to suppress PWM noise, fast enough to catch
// transient sag during current spikes.
// =============================================================================
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
public:
    BatteryMonitor(int adcPin);
    
    // Call once in setup() after Arduino init.
    void begin();
    
    // Single instantaneous read. Returns battery voltage in volts.
    float readVoltage();
    
    // Periodic update for the stats tracker. Call at ~100 Hz from main loop.
    // When stats tracking is OFF, this is a no-op.
    void update();
    
    // Stats tracking — useful for capturing battery sag during a servo move.
    void  resetStats();
    bool  isStatsTracking() const { return _tracking; }
    void  setStatsTracking(bool on);
    
    float minVoltage() const { return _vMin; }
    float maxVoltage() const { return _vMax; }
    float avgVoltage() const { return (_sampleCount > 0) ? (_vSum / _sampleCount) : 0.0f; }
    uint32_t sampleCount() const { return _sampleCount; }
    
    // Health classification based on current reading.
    enum class Health { Full, Nominal, Warning, Critical, Disconnected };
    Health health();
    const char* healthString();
    
private:
    int     _adcPin;
    
    // Stats state
    bool     _tracking    = false;
    float    _vMin        = 99.0f;
    float    _vMax        = 0.0f;
    float    _vSum        = 0.0f;
    uint32_t _sampleCount = 0;
    
    uint32_t _lastSampleMicros = 0;
    static constexpr uint32_t SAMPLE_INTERVAL_US = 10000;  // 100 Hz
};

#endif