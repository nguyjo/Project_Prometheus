// =============================================================================
// Buzzer.cpp
// =============================================================================
#include "Buzzer.h"

Buzzer::Buzzer(uint8_t pin, const char* name) : _pin(pin), _name(name) {
}

void Buzzer::begin() {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  _pinState = false;
  _mode = IDLE;
}

void Buzzer::writePin(bool state) {
  _pinState = state;
  digitalWrite(_pin, state ? HIGH : LOW);
}

void Buzzer::beepOnce(uint32_t durationMs) {
  _mode = ONCE;
  _onceDurationMs = durationMs;
  _stateChangeMs = millis();
  writePin(true);
}

void Buzzer::beepPattern(uint8_t count, uint32_t onMs, uint32_t offMs, uint32_t pauseMs) {
  if (count == 0) {
    silence();
    return;
  }
  _mode = PATTERN;
  _patternCount = count;
  _patternIndex = 0;
  _patternOnMs = onMs;
  _patternOffMs = offMs;
  _patternPauseMs = pauseMs;
  _inPause = false;
  _stateChangeMs = millis();
  writePin(true);
}

void Buzzer::setContinuous(bool on) {
  if (on) {
    _mode = CONTINUOUS;
    writePin(true);
  } else {
    silence();
  }
}

void Buzzer::silence() {
  _mode = IDLE;
  writePin(false);
}

void Buzzer::update() {
  uint32_t now = millis();
  uint32_t elapsed = now - _stateChangeMs;

  switch (_mode) {
    case IDLE:
    case CONTINUOUS:
      break;

    case ONCE:
      if (_pinState && elapsed >= _onceDurationMs) {
        writePin(false);
        _mode = IDLE;
      }
      break;

    case PATTERN:
      if (_inPause) {
        if (elapsed >= _patternPauseMs) {
          _inPause = false;
          _patternIndex = 0;
          _stateChangeMs = now;
          writePin(true);
        }
      } else if (_pinState) {
        if (elapsed >= _patternOnMs) {
          writePin(false);
          _stateChangeMs = now;
        }
      } else {
        if (elapsed >= _patternOffMs) {
          _patternIndex++;
          if (_patternIndex >= _patternCount) {
            _inPause = true;
            _stateChangeMs = now;
          } else {
            _stateChangeMs = now;
            writePin(true);
          }
        }
      }
      break;
  }
}