#include "ExternalConn.h"
#include "Config.h"

// Constructor
ExternalConn::ExternalConn(int detectPin) : _detectPin(detectPin) {
  isArmed = false;
  isLogging = false;
}

void ExternalConn::begin() {
  pinMode(_detectPin, INPUT_PULLUP);
  Serial4.begin(115200); 
  delay(100);
}

bool ExternalConn::isConnected() {
  // Returns true if the pin is bridged to GND
  return (digitalRead(_detectPin) == LOW);
}

void ExternalConn::update() {
  // Process any new command
  if (isConnected() && Serial4.available() > 0) {
    String cmd = Serial4.readStringUntil('\n');
    cmd.trim();          
    cmd.toUpperCase();   
    processCommand(cmd);
  }
}

void ExternalConn::processCommand(String cmd) {
  Serial4.print("\n>>> ROCKET RECEIVED: ");
  Serial4.println(cmd);

  if (cmd == "HELP") {
    Serial4.println("AVAILABLE COMMANDS:");
    Serial4.println("  HELP            - Show this menu");
    Serial4.println("  STATUS          - System status");
    Serial4.println("  CAL_GYRO        - Calibrate gyro bias");
    Serial4.println("  CAL_BARO        - Calibrate ground altitude");
    Serial4.println("  ARM             - Software arm (DO NOT USE on bench)");
    Serial4.println("  LOG_START       - Begin data logging");
    Serial4.println("  STAGE           - Report which stage firmware is loaded");
  }
  else if (cmd == "STATUS") {
    Serial4.println("AVIONICS HEALTHY. WAITING FOR CALIBRATION.");
  }
  else if (cmd == "ARM") {
    isArmed = true;
    Serial4.println("WARNING: SOFTWARE ARMED. READY FOR FLIGHT.");
  }
  else if (cmd == "LOG_START") {
    isLogging = true;
    Serial4.println("DATA LOGGING STARTED.");
  }
  else if (cmd == "STAGE") {
    Serial4.printf(">>> Stage: %s\n", Config::Build::STAGE_NAME);
    Serial4.printf(">>> Build: %s %s\n", Config::Build::BUILD_DATE, Config::Build::BUILD_TIME);
  }
  else {
    Serial4.println("ERROR: UNKNOWN COMMAND. TYPE 'HELP'.");
  }
}