// =============================================================================
// BoosterFlightStrategy.h
// =============================================================================
// Flight state machine for the BOOSTER stage of the Prometheus 2-stage rocket.
//
// FLIGHT PROFILE (from team operations doc):
//   T+0      Booster ignition (liftoff)
//   T+~3s    Booster burnout
//   T+~10s   Sustainer hot-stages — physical separation triggers booster's
//            drogue release mechanism at ~90 m/s (MECHANICAL, not firmware)
//            Booster reaches apogee shortly after, falls under drogue
//   Descent  Booster falls under drogue until 1,500 ft (457m AGL)
//   1,500ft  FIRMWARE fires PYRO0 (main parachute charge)
//   Ground   Touchdown — recovery buzzer turns on continuously
//
// STATE SEQUENCE:
//
//   PRE_LIFTOFF
//     ↓ (sustained accel > 2.5g for 100ms — booster ignition)
//   BOOST                                   [~3s, motor burning]
//     ↓ (sustained low accel for 200ms — booster burnout)
//   COAST                                   [hot-staging happens here]
//     ↓ (drogue mechanically released, no firmware action)
//   DESCENT_DROGUE                          [descending under drogue]
//     ↓ (altitude descends through 1,500 ft AGL)
//   MAIN_FIRED                              [fire PYRO0]
//     ↓ (fire pulse completes)
//   DESCENT_MAIN                            [descending under main]
//     ↓ (sustained low velocity at low alt for 5s)
//   TOUCHDOWN                               [recovery buzzer ON]
//
// IMPORTANT: The booster firmware does NOT fire an apogee charge. Apogee
// deployment (drogue release) is purely mechanical, triggered by the
// physical separation force when the sustainer hot-stages.
// =============================================================================

#ifndef BOOSTERFLIGHTSTRATEGY_H
#define BOOSTERFLIGHTSTRATEGY_H

#include "FlightStrategy.h"
#include <Arduino.h>

class StateEstimator;
class AccelerationSource;
class Barometer;
class PyroChannel;
class Buzzer;

class BoosterFlightStrategy : public FlightStrategy {
  public:
    BoosterFlightStrategy(
        StateEstimator& estimator,
        AccelerationSource& accelSrc,
        Barometer& baro,
        PyroChannel& mainPyro,
        Buzzer& uiBuzzer,
        Buzzer& recoveryBuzzer);

    void update() override;
    const char* currentStateName() const override;
    bool isFlightComplete() const override;

    // Telemetry accessors
    uint32_t liftoffTimestampMs() const { return _liftoffMs; }
    uint32_t burnoutTimestampMs() const { return _burnoutMs; }
    uint32_t mainFiredTimestampMs() const { return _mainFiredMs; }

    uint8_t currentStateId() const override;

  private:
    enum State {
      PRE_LIFTOFF,
      BOOST,
      COAST,
      DESCENT_DROGUE,
      MAIN_FIRED,
      DESCENT_MAIN,
      TOUCHDOWN
    };

    State _state = PRE_LIFTOFF;

    // Subsystem references
    StateEstimator&     _estimator;
    AccelerationSource& _accelSrc;
    Barometer&          _baro;
    PyroChannel&        _mainPyro;
    Buzzer&             _uiBuzzer;
    Buzzer&             _recoveryBuzzer;

    // State entry timestamps (for telemetry)
    uint32_t _liftoffMs    = 0;
    uint32_t _burnoutMs    = 0;
    uint32_t _mainFiredMs  = 0;
    uint32_t _touchdownMs  = 0;

    // Trigger latching (sustained-detection timers)
    uint32_t _liftoffCandidateMs   = 0;
    uint32_t _burnoutCandidateMs   = 0;
    uint32_t _touchdownCandidateMs = 0;

    // Maximum altitude seen so far — used to detect "we're descending"
    // (combined with velocity < 0). Apogee in the booster case isn't a
    // firmware event but we still track it for the COAST → DESCENT_DROGUE
    // transition.
    float _maxAltitudeSeen = 0.0f;

    // Helpers
    void enterBoost();
    void enterCoast();
    void enterDescentDrogue();
    void enterMainFired();
    void enterDescentMain();
    void enterTouchdown();

    void updatePreLiftoff();
    void updateBoost();
    void updateCoast();
    void updateDescentDrogue();
    void updateMainFired();
    void updateDescentMain();
    void updateTouchdown();
};

#endif