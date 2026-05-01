#include <Arduino.h>
#include "ExternalConn.h"

// Instantiate the ExternalConn object on Pin 18
ExternalConn externalConn(18); 

void setup() {
  externalConn.begin();
  // Initialize other sensors here...
}

void loop() {
  // 1. Always check for Ground Support Equipment commands
  externalConn.update();

  // 2. Flight Logic
  if (externalConn.isConnected()) {
    // --- GROUND MODE ---
    // The rocket is on the pad. Do nothing but listen to the laptop.
    
  } else {
    // --- FLIGHT MODE ---
    // The umbilical is unplugged. 
    
    // Check if we are allowed to fly
    if (externalConn.isArmed) {
        // Run Sensor Fusion
        // Run Apogee Detection State Machine
    }

    if (externalConn.isLogging) {
        // Save data to QSPI Flash
    }
  }
}