// =============================================================================
// FlightStrategy.h
// =============================================================================
// Abstract interface for stage-specific flight state machines.
//
// Two concrete implementations:
//   - BoosterFlightStrategy:   simple 2-stage descent (drogue+main), main pyro only
//   - SustainerFlightStrategy: separation/tilt-check/ignition + main pyro
//
// Each strategy owns its own state machine. Main loop calls update() every
// iteration during flight.
// =============================================================================

#ifndef FLIGHTSTRATEGY_H
#define FLIGHTSTRATEGY_H

#include <stdint.h>

class FlightStrategy {
  public:
    virtual ~FlightStrategy() = default;

    // Called every main loop iteration when in flight mode.
    virtual void update() = 0;

    // Returns a human-readable name of the current state (for telemetry/logs).
    virtual const char* currentStateName() const = 0;

    // Returns true if flight is complete (touchdown).
    virtual bool isFlightComplete() const = 0;

    virtual uint8_t currentStateId() const = 0;
};

#endif