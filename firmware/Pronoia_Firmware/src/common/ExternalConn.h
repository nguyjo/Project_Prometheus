#ifndef EXTERNALCONN_H
#define EXTERNALCONN_H

#include <Arduino.h>

class ExternalConn {
  public:
    ExternalConn(int detectPin); // Constructor with just the External Connection detect pin

    // Methods
    void begin();
    void update();
    bool isConnected();

    // Flight Flags (Public so main.cpp can read them)
    bool isArmed;
    bool isLogging;

  private:
    int _detectPin;

    void processCommand(String cmd);
};

#endif