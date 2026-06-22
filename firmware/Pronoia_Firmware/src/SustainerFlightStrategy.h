// =============================================================================
// SustainerFlightStrategy.h
// =============================================================================
// Flight state machine for the SUSTAINER stage of the Prometheus rocket.
//
// FLIGHT PROFILE:
//   T+0       Booster ignition (sustainer along for the ride)
//   T+~3s     Booster burnout
//   T+~10s    Sustainer hot-stages — physical separation
//   T+~10s    If tilt < 45° AND no rotation fault: FIRE sustainer ignitor (PYRO0)
//             If tilt >= 45° OR rotation fault: ABORT, never light sustainer
//   T+~12s    Sustainer burnout (~1.7s burn)
//   T+~26s    Sustainer apogee (motor ejection deploys drogue)
//   Descent   Sustainer falls under drogue
//   1,500 ft  Firmware fires sustainer main parachute (PYRO1)
//   Ground    Touchdown — recovery buzzer ON
//
// STATE SEQUENCE: (see SustainerFlightStrategy.cpp for details)
//
// ABORT BRANCH: entered if tilt check fails. The interstage friction-fits,
// so the rocket may still separate during descent due to drag. Each stage
// fires its own main at 1,500 ft. Recovery is degraded but probably
// survivable.
//
// RATE-DAMPING CONTROLLER LIFECYCLE (hot-staging design):
//   - Disabled through STAGE1_BOOST and SEPARATION_COAST. During these
//     states the sustainer is still attached to the booster; its dynamics
//     are completely different (mass, inertia, fin authority), and running
//     a sustainer-tuned controller on combined-vehicle dynamics is wrong.
//   - Enabled on entry to STAGE2_BOOST — sustainer thrust is confirmed,
//     separation has occurred via the ignition itself, and the rocket is
//     now alone with well-known dynamics.
//   - Remains enabled through COAST.
//   - Disabled on entry to POST_APOGEE_DESCENT (fins ineffective during
//     descent under drogue, and we want them centered).
//   - Disabled in all four ABORT_* enter functions (defensive; abort path
//     never enables it in the first place).
// =============================================================================

#ifndef SUSTAINERFLIGHTSTRATEGY_H
#define SUSTAINERFLIGHTSTRATEGY_H

#include "FlightStrategy.h"
#include <Arduino.h>

class StateEstimator;
class AccelerationSource;
class Barometer;
class IMU;
class TiltEstimator;
class PyroChannel;
class Buzzer;
class RateDampingController;

class SustainerFlightStrategy : public FlightStrategy {
  public:
    SustainerFlightStrategy(
        StateEstimator& estimator,
        AccelerationSource& accelSrc,
        Barometer& baro,
        IMU& imu,
        TiltEstimator& tiltEst,
        PyroChannel& ignitorPyro,
        PyroChannel& mainPyro,
        Buzzer& uiBuzzer,
        Buzzer& recoveryBuzzer,
        RateDampingController& rateDamper);

    void update() override;
    const char* currentStateName() const override;
    uint8_t currentStateId() const override;
    bool isFlightComplete() const override;

    // Diagnostic accessors for telemetry/logs
    uint32_t liftoffTimestampMs()    const { return _liftoffMs; }
    uint32_t stage1BurnoutMs()       const { return _stage1BurnoutMs; }
    uint32_t ignitionTimestampMs()   const { return _ignitionMs; }
    uint32_t stage2BurnoutMs()       const { return _stage2BurnoutMs; }
    bool     didAbort()              const { return _state == ABORT_DESCENT_DROGUE
                                                 || _state == ABORT_MAIN_FIRED
                                                 || _state == ABORT_DESCENT_MAIN
                                                 || _state == ABORT_TOUCHDOWN; }
    float    tiltAtIgnitionDeg()     const { return _tiltAtIgnitionDeg; }

  private:
    enum State {
      PRE_LIFTOFF,
      STAGE1_BOOST,
      SEPARATION_COAST,
      TILT_CHECK,
      SUSTAINER_IGNITION,
      STAGE2_BOOST,
      COAST,
      POST_APOGEE_DESCENT,    // Renamed from DESCENT_DROGUE — drogue MAY be deployed
      MAIN_FIRED,
      DESCENT_UNDER_MAIN,
      TOUCHDOWN,

      // Abort branch — tilt failed or sustainer didn't ignite
      ABORT_DESCENT_DROGUE,
      ABORT_MAIN_FIRED,
      ABORT_DESCENT_MAIN,
      ABORT_TOUCHDOWN
    };

    State _state = PRE_LIFTOFF;

    // Subsystem references
    StateEstimator&        _estimator;
    AccelerationSource&    _accelSrc;
    Barometer&             _baro;
    IMU&                   _imu;
    TiltEstimator&         _tiltEst;
    PyroChannel&           _ignitorPyro;
    PyroChannel&           _mainPyro;
    Buzzer&                _uiBuzzer;
    Buzzer&                _recoveryBuzzer;
    RateDampingController& _rateDamper;

    // State entry timestamps
    uint32_t _liftoffMs        = 0;
    uint32_t _stage1BurnoutMs  = 0;
    uint32_t _ignitionMs       = 0;
    uint32_t _stage2BurnoutMs  = 0;
    uint32_t _mainFiredMs      = 0;
    uint32_t _touchdownMs      = 0;

    // Sustained-detection timers
    uint32_t _liftoffCandidateMs       = 0;
    uint32_t _stage1BurnoutCandidateMs = 0;
    uint32_t _stage2LiftoffCandidateMs = 0;
    uint32_t _stage2BurnoutCandidateMs = 0;
    uint32_t _touchdownCandidateMs     = 0;

    // Tilt-check window tracking
    uint32_t _tiltCheckStartMs = 0;

    // Max altitude tracking
    float _maxAltitudeSeen = 0.0f;

    // Diagnostic: tilt angle at the moment of TILT_CHECK
    float _tiltAtIgnitionDeg = -1.0f;

    // State entry callbacks
    void enterStage1Boost();
    void enterSeparationCoast();
    void enterTiltCheck();
    void enterSustainerIgnition();
    void enterStage2Boost();
    void enterCoast();
    void enterPostApogeeDescent();
    void enterMainFired();
    void enterDescentUnderMain();
    void enterTouchdown();

    void enterAbortDescentDrogue();
    void enterAbortMainFired();
    void enterAbortDescentMain();
    void enterAbortTouchdown();

    // State update functions
    void updatePreLiftoff();
    void updateStage1Boost();
    void updateSeparationCoast();
    void updateTiltCheck();
    void updateSustainerIgnition();
    void updateStage2Boost();
    void updateCoast();
    void updatePostApogeeDescent();
    void updateMainFired();
    void updateDescentUnderMain();
    void updateTouchdown();

    void updateAbortDescentDrogue();
    void updateAbortMainFired();
    void updateAbortDescentMain();
    void updateAbortTouchdown();
};

#endif