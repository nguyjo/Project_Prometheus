// FlightStrategy.h
#ifndef FLIGHTSTRATEGY_H
#define FLIGHTSTRATEGY_H

class FlightStrategy {
  public:
    virtual ~FlightStrategy() = default;

    // Called every loop iteration during flight
    virtual void update(float dt_s) = 0;

    // Called when liftoff is detected
    virtual void onLiftoff() = 0;

    // Called when motor burnout is detected
    virtual void onBurnout() = 0;

    // Called when apogee is detected
    virtual void onApogee() = 0;

    // Called when main deployment altitude is reached
    virtual void onMainDeploy() = 0;
};

#endif