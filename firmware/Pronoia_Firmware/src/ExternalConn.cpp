#include "ExternalConn.h"

// Constructor
ExternalConn::ExternalConn(int detectPin, IMU& imu, HighGAccel& highG,
                           AccelerationSource& accelSrc)
  : _detectPin(detectPin), _imu(imu), _highG(highG), _accelSrc(accelSrc) {
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
  // Always service streaming if active, regardless of new commands
  if (_streaming && (millis() - _lastStreamPrintMs >= STREAM_INTERVAL_MS)) {
    streamSensorReadings();
    _lastStreamPrintMs = millis();
  }

  // Process any new command
  if (isConnected() && Serial4.available() > 0) {
    String cmd = Serial4.readStringUntil('\n');
    cmd.trim();          
    cmd.toUpperCase();   
    processCommand(cmd);
  }
}

void ExternalConn::processCommand(String cmd) {
    // Any command stops streaming except for STREAM_SENSORS itself
  if (_streaming && cmd != "STREAM_SENSORS") {
    _streaming = false;
    Serial4.println("\n>>> STREAM STOPPED");
  }

  Serial4.print("\n>>> ROCKET RECEIVED: ");
  Serial4.println(cmd);

  if (cmd == "HELP") {
    Serial4.println("AVAILABLE COMMANDS:");
    Serial4.println("  HELP            - Show this menu");
    Serial4.println("  STATUS          - System status");
    Serial4.println("  STREAM_SENSORS  - Begin streaming sensor data (any cmd to stop)");
    Serial4.println("  STOP            - Stop any active streaming");
    Serial4.println("  CAL_GYRO        - Calibrate gyro bias");
    Serial4.println("  CAL_BARO        - Calibrate ground altitude");
    Serial4.println("  ARM             - Software arm (DO NOT USE on bench)");
    Serial4.println("  LOG_START       - Begin data logging");
  }
  else if (cmd == "STATUS") {
    Serial4.println("AVIONICS HEALTHY. WAITING FOR CALIBRATION.");
  }
  else if (cmd == "STREAM_SENSORS") {
    _streaming = true;
    _lastStreamPrintMs = 0;  // Print immediately on next update()
    Serial4.println(">>> STREAMING SENSORS @ 20Hz. Send any command to stop.");
    Serial4.println(">>> Format: raw chip values + rocket-frame accessors");
  }
  else if (cmd == "STOP") {
    // Streaming already stopped above — this is just an explicit acknowledge
    Serial4.println(">>> Stopped.");
  }
  else if (cmd == "ARM") {
    isArmed = true;
    Serial4.println("WARNING: SOFTWARE ARMED. READY FOR FLIGHT.");
  }
  else if (cmd == "LOG_START") {
    isLogging = true;
    Serial4.println("DATA LOGGING STARTED.");
  }
  else {
    Serial4.println("ERROR: UNKNOWN COMMAND. TYPE 'HELP'.");
  }
}

void ExternalConn::streamSensorReadings() {
  // Single-line streaming format for axis verification.
  // Layout is designed so each sensor's three axes line up vertically
  // when scrolling — makes it easy to spot which axis is reading +1g
  // when the rocket is held in different orientations.
  Serial4.printf(
    "IMU[X:%+6.3f Y:%+6.3f Z:%+6.3f g] "
    "GYR[X:%+7.2f Y:%+7.2f Z:%+7.2f dps] "
    "KX134[X:%+6.3f Y:%+6.3f Z:%+6.3f g] | "
    "Rkt[ax:%+6.3f l1:%+6.3f l2:%+6.3f g | "
    "roll:%+7.2f pitch:%+7.2f yaw:%+7.2f dps] | "
    "Fused:%+7.2f m/s^2 src:%s\n",
    _imu.accelX, _imu.accelY, _imu.accelZ,
    _imu.gyroX, _imu.gyroY, _imu.gyroZ,
    _highG.accelX, _highG.accelY, _highG.accelZ,
    _imu.axialAccelG(), _imu.lat1AccelG(), _imu.lat2AccelG(),
    _imu.rollRateDps(), _imu.pitchRateDps(), _imu.yawRateDps(),
    _accelSrc.getAxialAccelMps2(),
    _accelSrc.isUsingHighG() ? "KX134" : "IMU"
  );
}