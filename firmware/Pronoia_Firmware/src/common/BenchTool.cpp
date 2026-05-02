#include "BenchTool.h"
#include "Config.h"

BenchTool::BenchTool(IMU& imu, HighGAccel& highG, AccelerationSource& accelSrc, Barometer& baro)
  : _imu(imu), _highG(highG), _accelSrc(accelSrc), _baro(baro) {
}

void BenchTool::begin() {
  // Note: Serial.begin() is called in main setup() — we don't duplicate it here
  // because Serial is the USB port and is shared with debug print statements.
}

void BenchTool::update() {
  // Service streaming if active
  if (_streaming && (millis() - _lastStreamPrintMs >= STREAM_INTERVAL_MS)) {
    streamSensorReadings();
    _lastStreamPrintMs = millis();
  }

  // Process any new command from USB
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    processCommand(cmd);
  }
}

void BenchTool::processCommand(String cmd) {
  // Any command stops streaming except STREAM_SENSORS itself
  if (_streaming && cmd != "STREAM_SENSORS") {
    _streaming = false;
    Serial.println("\n>>> STREAM STOPPED");
  }

  Serial.print("\n>>> BENCH RECEIVED: ");
  Serial.println(cmd);

  if (cmd == "HELP") {
    printHelp();
  }
  else if (cmd == "STREAM_SENSORS") {
    _streaming = true;
    _lastStreamPrintMs = 0;
    Serial.println(">>> STREAMING SENSORS @ 20Hz. Send any command to stop.");
    Serial.println(">>> RAW = chip frame (X/Y/Z labels)");
    Serial.println(">>> RKT = rocket frame via accessors (axial / roll-pitch-yaw)");
    Serial.println(">>> Compare RAW to RKT during physical tests to verify accessor signs.");
  }
  else if (cmd == "STOP") {
    Serial.println(">>> Stopped.");
  }
  else if (cmd == "STAGE") {
    Serial.printf(">>> Stage: %s\n", Config::Build::STAGE_NAME);
    Serial.printf(">>> Build: %s %s\n", Config::Build::BUILD_DATE, Config::Build::BUILD_TIME);
  }
  else {
    Serial.println("ERROR: UNKNOWN COMMAND. TYPE 'HELP'.");
  }
}

void BenchTool::printHelp() {
  Serial.println("==== BENCH DIAGNOSTIC COMMANDS ====");
  Serial.println("  HELP            - Show this menu");
  Serial.println("  STREAM_SENSORS  - Stream raw + rocket-frame sensor data");
  Serial.println("  STOP            - Stop any active streaming");
  Serial.println("  STAGE           - Report which stage firmware is loaded");
  Serial.println("====================================");
}

void BenchTool::streamSensorReadings() {
  Serial.printf(
    "[%6lu] "
    "IMU_Rkt[ax:%+6.3f g | roll:%+7.2f pitch:%+7.2f yaw:%+7.2f dps] | "
    "KX_Rkt[ax:%+6.3f g] | "
    "Fused:%+7.2f m/s^2 src:%s | "
    "BARO[AGL:%+7.2f smooth:%+7.2f vel:%+6.2f m/s | maxAlt:%+7.2f T:%5.1fC P:%7.2fmbar]\n",
    millis(),
    _imu.axialAccelG(),
    _imu.rollRateDps(), _imu.pitchRateDps(), _imu.yawRateDps(),
    _highG.axialAccelG(),
    _accelSrc.getAxialAccelMps2(),
    _accelSrc.isUsingHighG() ? "KX134" : "IMU",
    _baro.altitude_AGL,
    _baro.smoothedAltitude_AGL,
    _baro.velocity_mps,
    _baro.maxAltitude_AGL,
    _baro.temp_C,
    _baro.pressure_Mbar
  );
}