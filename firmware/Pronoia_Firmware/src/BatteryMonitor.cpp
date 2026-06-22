// =============================================================================
// BatteryMonitor.cpp
// =============================================================================
#include "BatteryMonitor.h"
#include "Config.h"

BatteryMonitor::BatteryMonitor(int adcPin)
    : _adcPin(adcPin)
{}

void BatteryMonitor::begin() {
    pinMode(_adcPin, INPUT);
    analogReadResolution(12);  // 0..4095 instead of default 0..1023
    
    // Throw away the first few samples — ADC stabilization after pinMode.
    for (int i = 0; i < 4; i++) {
        analogRead(_adcPin);
    }
}

float BatteryMonitor::readVoltage() {
    // Average 4 samples for noise rejection. ADC is fast (~5 µs per read),
    // so 4 samples is ~20 µs total — negligible loop time.
    uint32_t raw = 0;
    for (int i = 0; i < 4; i++) {
        raw += analogRead(_adcPin);
    }
    float raw_avg = raw / 4.0f;
    
    float v_monitor = (raw_avg / Config::Power::ADC_MAX_COUNTS)
                      * Config::Power::ADC_REFERENCE_V;
    float v_battery = v_monitor * Config::Power::VBATT_DIVIDER_RATIO;
    return v_battery;
}

void BatteryMonitor::update() {
    if (!_tracking) return;
    
    const uint32_t now = micros();
    if (now - _lastSampleMicros < SAMPLE_INTERVAL_US) return;
    _lastSampleMicros = now;
    
    float v = readVoltage();
    if (v < _vMin) _vMin = v;
    if (v > _vMax) _vMax = v;
    _vSum += v;
    _sampleCount++;
}

void BatteryMonitor::resetStats() {
    _vMin = 99.0f;
    _vMax = 0.0f;
    _vSum = 0.0f;
    _sampleCount = 0;
    _lastSampleMicros = micros();
}

void BatteryMonitor::setStatsTracking(bool on) {
    if (on && !_tracking) {
        resetStats();
    }
    _tracking = on;
}

BatteryMonitor::Health BatteryMonitor::health() {
    float v = readVoltage();
    if (v < 1.0f)                                  return Health::Disconnected;
    if (v < Config::Power::VBATT_CRITICAL_V)       return Health::Critical;
    if (v < Config::Power::VBATT_WARN_V)           return Health::Warning;
    if (v < Config::Power::VBATT_FULL_V - 0.2f)    return Health::Nominal;
    return Health::Full;
}

const char* BatteryMonitor::healthString() {
    switch (health()) {
        case Health::Full:         return "FULL";
        case Health::Nominal:      return "NOMINAL";
        case Health::Warning:      return "WARNING";
        case Health::Critical:     return "CRITICAL";
        case Health::Disconnected: return "DISCONNECTED";
    }
    return "?";
}