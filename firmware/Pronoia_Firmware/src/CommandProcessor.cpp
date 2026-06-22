// =============================================================================
// CommandProcessor.cpp
// =============================================================================
#include "CommandProcessor.h"
#include "Config.h"
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "CommandProcessor.h"
#include "FlashLogger.h"
#include "RateDampingController.h"
#include "BatteryMonitor.h"
#include "TiltEstimator.h"

CommandProcessor::CommandProcessor(
    PyroChannel& pyro0, PyroChannel& pyro1,
    PyroChannel& pyro2, PyroChannel& pyro3,
    Buzzer& uiBuzzer, Buzzer& recoveryBuzzer,
    IMU& imu, HighGAccel& highG,
    AccelerationSource& accelSrc, Barometer& baro,
    FlashLogger& flashLogger,
    BatteryMonitor& batteryMonitor,
    RateDampingController* rateDamper,
    TiltEstimator* tiltEstimator)
  : _pyro0(pyro0), _pyro1(pyro1), _pyro2(pyro2), _pyro3(pyro3),
    _uiBuzzer(uiBuzzer), _recoveryBuzzer(recoveryBuzzer),
    _imu(imu), _highG(highG), _accelSrc(accelSrc), _baro(baro),
    _batteryMonitor(batteryMonitor),
    _flashLogger(flashLogger), _rateDamper(rateDamper), _tiltEst(tiltEstimator)
{
  _pyros[0] = &_pyro0;
  _pyros[1] = &_pyro1;
  _pyros[2] = &_pyro2;
  _pyros[3] = &_pyro3;
}

// -----------------------------------------------------------------------------
// Periodic service
// -----------------------------------------------------------------------------
void CommandProcessor::update() {
  // Auto-disarm if a pyro's arm window has expired.
  if (_armedPyroIndex >= 0 &&
      (millis() - _armedPyroAtMs > Config::Pyro::ARM_TIMEOUT_MS)) {
    _pyros[_armedPyroIndex]->setArmed(false);
    _armedPyroIndex = -1;
  }
}

void CommandProcessor::serviceStreaming(Stream& out) {
  // Only stream to the Stream that initiated streaming.
  if (!_streaming || _streamingTarget != &out) return;

  if (millis() - _lastStreamPrintMs >= STREAM_INTERVAL_MS) {
    streamSensors(out);
    _lastStreamPrintMs = millis();
  }

  // ----- NEW: Tilt streaming -----
  if (_tiltStreaming && _tiltStreamingTarget == &out && _tiltEst != nullptr) {
    if (millis() - _lastTiltStreamMs >= TILT_STREAM_INTERVAL_MS) {
      _lastTiltStreamMs = millis();
      out.print(F("[TILT] tilt="));
      out.print(_tiltEst->tiltDeg(), 1);
      out.print(F("deg  max="));
      out.print(_tiltEst->maxTiltDeg(), 1);
      out.print(F("deg  fault="));
      out.println(_tiltEst->rotationFault() ? F("YES") : F("no"));
    }
  }
}

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

bool CommandProcessor::anyChannelHasContinuity() const {
  for (int i = 0; i < 4; i++) {
    if (_pyros[i]->hasContinuity()) return true;
  }
  return false;
}

int8_t CommandProcessor::parsePyroIndex(const String& cmd, const String& prefix) const {
  if (!cmd.startsWith(prefix)) return -1;
  if (cmd.length() != prefix.length() + 1) return -1;
  char c = cmd.charAt(prefix.length());
  if (c < '0' || c > '3') return -1;
  return c - '0';
}

void CommandProcessor::disarmPyro(Stream& out, bool quiet) {
  if (_armedPyroIndex >= 0) {
    _pyros[_armedPyroIndex]->setArmed(false);
    _armedPyroIndex = -1;
    if (!quiet) out.println(F("[DISARMED]"));
  } else if (!quiet) {
    out.println(F("[ALREADY DISARMED]"));
  }
}

void CommandProcessor::emergencyStop(Stream& out) {
  for (int i = 0; i < 4; i++) {
    _pyros[i]->forceOff();
  }
  _armedPyroIndex = -1;
  _uiBuzzer.silence();
  _recoveryBuzzer.silence();
  // Also stop any active streaming
  _streaming = false;
  _streamingTarget = nullptr;
  out.println(F("[ESTOP] All pyros forced off. Buzzers silenced. Disarmed."));
}

// -----------------------------------------------------------------------------
// Reports
// -----------------------------------------------------------------------------

void CommandProcessor::printStatus(Stream& out) {
  out.println();
  out.println(F("---- PYRO STATUS ----"));
  for (int i = 0; i < 4; i++) {
    out.print(F("  "));
    out.print(_pyros[i]->name());
    out.print(F("  cont="));
    out.print(_pyros[i]->hasContinuity() ? F("YES") : F("NO "));
    out.print(F("   firing="));
    out.println(_pyros[i]->isFiring() ? F("YES") : F("NO"));
  }
  out.print(F("Armed pyro:    "));
  if (_armedPyroIndex < 0) {
    out.println(F("NONE"));
  } else {
    uint32_t elapsed = millis() - _armedPyroAtMs;
    uint32_t remaining = (elapsed < Config::Pyro::ARM_TIMEOUT_MS)
                       ? (Config::Pyro::ARM_TIMEOUT_MS - elapsed) : 0;
    out.print(F("PYRO"));
    out.print(_armedPyroIndex);
    out.print(F(" ("));
    out.print(remaining);
    out.println(F(" ms remaining)"));
  }
  out.print(F("Flight armed:  "));
  out.println(_flightArmed ? F("YES") : F("NO"));
  out.print(F("Logging:       "));
  out.println(_logging ? F("YES") : F("NO"));
  out.print(F("FingerTech:    "));
  if (anyChannelHasContinuity()) {
    out.println(F("LIKELY ON (at least one channel sensing)"));
  } else {
    out.println(F("UNKNOWN (no continuity sensed)"));
  }
  out.println(F("---------------------"));
  out.println();
}

void CommandProcessor::printReadyCheck(Stream& out) {
  out.println();
  out.println(F("---- READY CHECK ----"));
  if (anyChannelHasContinuity()) {
    out.println(F("Continuity sensed on at least one channel."));
    out.println(F("=> FingerTech is ON. Firing path is LIVE."));
    out.println(F("=> Continuity status is meaningful."));
  } else {
    out.println(F("No continuity sensed on any channel."));
    out.println(F("=> EITHER FingerTech is OFF (firing path safe)"));
    out.println(F("=> OR FingerTech is ON but no loads/ematches connected"));
    out.println(F("=> Verify visually before proceeding."));
  }
  out.println(F("---------------------"));
  out.println();
}

void CommandProcessor::printHelp(Stream& out) {
  out.println();
  out.println(F("==== PROMETHEUS COMMANDS ===="));
  out.println(F("  HELP            - Show this menu"));
  out.println(F("  STATUS          - Pyro continuity, arming, FingerTech state"));
  out.println(F("  READY_CHECK     - Detect whether FingerTech is ON"));
  out.println(F("  STAGE           - Report which stage firmware is loaded"));
  out.println();
  out.println(F("  --- Calibration ---"));
  out.println(F("  CAL_ALL         - Calibrate gyro + barometer + tilt (~4 sec)"));
  out.println(F("  CAL_BARO        - Barometer ground reference only"));
  out.println();
  out.println(F("  --- Buzzers ---"));
  out.println(F("  BUZZ_UI         - Single beep on UI buzzer"));
  out.println(F("  BUZZ_REC        - Beep on recovery buzzer"));
  out.println(F("  REC_ON          - Recovery buzzer continuous ON"));
  out.println(F("  REC_OFF         - Recovery buzzer OFF"));
  out.println(F("  PATTERN_n       - UI buzzer beeps n times (n=1..9)"));
  out.println();
  out.println(F("  --- Battery monitor ---"));
  out.println(F("  BATT              - Print current battery voltage"));
  out.println(F("  BATT_STREAM_ON    - Start stats tracking (min/max/avg)"));
  out.println(F("  BATT_STREAM_OFF   - Stop stats tracking"));
  out.println(F("  BATT_STATS        - Print min/max/avg and sag"));
  out.println(F("  BATT_RESET        - Clear stats counters"));
  out.println();
  out.println(F("  --- Pyros (two-step) ---"));
  out.println(F("  ARM_PYRO_n      - Arm pyro n (n=0..3), 5sec window"));
  out.println(F("  FIRE_PYRO_n     - Fire armed pyro n (must ARM first)"));
  out.println(F("  DISARM          - Cancel pending pyro arm"));
  out.println(F("  ESTOP           - Force all pyros off, silence buzzers"));
  out.println();
  out.println(F("  --- Flight ---"));
  out.println(F("  ARM             - Set flight-armed flag (autonomous flight enable)"));
  out.println(F("  DISARM_FLIGHT   - Clear flight-armed flag"));
  out.println(F("  LOG_START       - Begin data logging"));
  out.println(F("  LOG_STOP        - Stop data logging"));
  out.println();
  out.println(F("  --- Diagnostics ---"));
  out.println(F("  STREAM_SENSORS  - Stream sensor data @ 20 Hz to this interface"));
  out.println(F("  STOP            - Stop any active streaming"));
  out.println();
  out.println(F("  --- Tilt estimator (sustainer build only) ---"));
  out.println(F("  TILT_STATUS       - Print current tilt, max, fault, quaternion"));
  out.println(F("  TILT_RESET        - Reset to identity (rocket defined as vertical)"));
  out.println(F("  TILT_STREAM_ON    - Continuous 5 Hz tilt output"));
  out.println(F("  TILT_STREAM_OFF   - Stop streaming"));
  out.println();
  out.println(F("CIRCUIT NOTES:"));
  out.println(F("  - Continuity sense requires FingerTech ON"));
  out.println(F("  - Recovery buzzer is upstream of FingerTech (always testable)"));
  out.println(F("  - UI buzzer runs on +3.3V (always testable)"));
  out.println(F("============================="));
  out.println();
  out.println(F("  --- Canard bench test (sustainer build only) ---"));
  out.println(F("  CANARD_BENCH_ON     - Suspend controller, enable raw servo writes"));
  out.println(F("  CANARD_BENCH_OFF    - Resume controller (centers fins)"));
  out.println(F("  CANARD_TEST X NNN   - Write angle NNN (65..115) to servo X (A|B|C|D)"));
  out.println(F("  CANARD_TRIM_GET     - Display current trim offsets"));
  out.println(F("  CANARD_TRIM_SET X N - Set trim for servo X (A|B|C|D) to N deg (-7..7)"));
  out.println(F("  CANARD_TRIM_CLEAR   - Reset all trims to zero"));
}

// -----------------------------------------------------------------------------
// Sensor streaming
// -----------------------------------------------------------------------------

void CommandProcessor::streamSensors(Stream& out) {
  out.printf(
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

// -----------------------------------------------------------------------------
// Main command dispatch
// -----------------------------------------------------------------------------

void CommandProcessor::process(const String& cmd, Stream& out) {
  // Auto-stop streaming when ANY other command arrives. We compare against
  // STREAM_SENSORS so that re-sending the same command doesn't kill its own
  // stream.
  if (_streaming && cmd != "STREAM_SENSORS") {
    _streaming = false;
    _streamingTarget = nullptr;
    out.println(F("\n>>> STREAM STOPPED"));
  }

  out.print(F(">>> RX: "));
  out.println(cmd);

  // Defense in depth: if a non-status, non-fire command arrives while a pyro
  // is armed, auto-disarm. Prevents stale arming state from being exploitable
  // by an unrelated command sequence.
  bool isFireCmd   = cmd.startsWith("FIRE_PYRO_");
  bool isStatusCmd = (cmd == "STATUS" || cmd == "CONT" || cmd == "READY_CHECK");
  if (_armedPyroIndex >= 0 && !isFireCmd && !isStatusCmd && cmd != "DISARM") {
    int8_t prev = _armedPyroIndex;
    disarmPyro(out, true);
    out.print(F("[AUTO-DISARM] PYRO"));
    out.print(prev);
    out.println(F(" (non-fire command received while armed)"));
  }

  // ----- General -----
  if (cmd == "HELP") {
    printHelp(out);
  }
  else if (cmd == "STATUS" || cmd == "CONT") {
    printStatus(out);
  }
  else if (cmd == "READY_CHECK") {
    printReadyCheck(out);
  }
  else if (cmd == "STAGE") {
    out.printf(">>> Stage: %s\n", Config::Build::STAGE_NAME);
    out.printf(">>> Build: %s %s\n", Config::Build::BUILD_DATE, Config::Build::BUILD_TIME);
  }

  else if (cmd == "CAL_BARO") {
    out.println(F("Calibrating barometer... (3 sec)"));
    if (_baro.calibrateBaro(50)) {
      out.print(F("[OK] Ground reference: "));
      out.print(_baro.groundAltitude_MSL);
      out.println(F(" m MSL"));
    } else {
      out.println(F("[ERR] Barometer calibration failed."));
    }
  }

  // ----- All-sensors calibration -----
  else if (cmd == "CAL_ALL") {
    if (_flightArmed) {
      out.println(F("[ERR] Cannot calibrate while flight is armed."));
      out.println(F("      Send DISARM_FLIGHT first."));
      return;
    }

    out.println(F("================================================"));
    out.println(F("[CAL_ALL] Full sensor calibration starting."));
    out.println(F("================================================"));
    out.println(F("REQUIREMENTS:"));
    out.println(F("  - Rocket must be STATIONARY on the stand/rail"));
    out.println(F("  - Rocket must be VERTICAL (orientation that defines 'no tilt')"));
    out.println(F("  - Total time: ~4 seconds. Do not move rocket."));
    out.println();

    // Audible warning: 2 short beeps = calibration starting
    _uiBuzzer.beepOnce(150);
    delay(200);
    _uiBuzzer.beepOnce(150);
    delay(500);

    // ----- Phase 1: Gyro bias -----
    out.print(F("[CAL_ALL] Phase 1/3: Gyro bias (500 samples, ~2.5s)... "));
    out.flush();
    _imu.calibrateGyro(500);
    out.println(F("done."));
    out.print(F("           Bias dps: pitch="));
    out.print(_imu.gyroOffsetX(), 4);
    out.print(F(" roll="));
    out.print(_imu.gyroOffsetY(), 4);
    out.print(F(" yaw="));
    out.println(_imu.gyroOffsetZ(), 4);

    // Sanity-check the biases. If any axis is >20 dps, something is wrong:
    // either the rocket moved during calibration, or there's a hardware fault.
    const float biasMax = max(max(fabsf(_imu.gyroOffsetX()),
                                  fabsf(_imu.gyroOffsetY())),
                              fabsf(_imu.gyroOffsetZ()));
    if (biasMax > 20.0f) {
      out.println(F("           [WARN] Bias > 20 dps detected."));
      out.println(F("                  Was the rocket moving? Re-run CAL_ALL."));
    }

    // ----- Phase 2: Barometer ground reference -----
    out.print(F("[CAL_ALL] Phase 2/3: Barometer ground ref (50 samples)... "));
    out.flush();
    if (_baro.calibrateBaro(50)) {
      out.println(F("done."));
      out.print(F("           Ground altitude: "));
      out.print(_baro.groundAltitude_MSL, 2);
      out.println(F(" m MSL"));
    } else {
      out.println(F("FAILED."));
      out.println(F("           [ERR] Barometer calibration failed. Sensor issue?"));
    }

    // ----- Phase 3: Tilt estimator reset -----
    if (_tiltEst != nullptr) {
      out.print(F("[CAL_ALL] Phase 3/3: Tilt estimator reset... "));
      _tiltEst->reset();
      out.println(F("done."));
      out.println(F("           Quaternion = identity. Rocket = vertical."));
    } else {
      out.println(F("[CAL_ALL] Phase 3/3: SKIPPED (no tilt estimator on this build)"));
    }

    // Audible confirmation: 3 short beeps = calibration complete
    delay(200);
    _uiBuzzer.beepOnce(100);
    delay(150);
    _uiBuzzer.beepOnce(100);
    delay(150);
    _uiBuzzer.beepOnce(100);

    out.println();
    out.println(F("================================================"));
    out.println(F("[CAL_ALL] Calibration complete. Rocket may be moved."));
    out.println(F("          Run TILT_STATUS to verify ~0 deg tilt."));
    out.println(F("================================================"));
  }

  // ----- Buzzers -----
  else if (cmd == "BUZZ_UI") {
    _uiBuzzer.beepOnce(300);
    out.println(F("[OK] UI buzzer beep"));
  }
  else if (cmd == "BUZZ_REC") {
    _recoveryBuzzer.beepOnce(2000);  // slow-pulse buzzer needs ~2s for audible output
    out.println(F("[OK] Recovery buzzer beep (2 sec)"));
  }
  else if (cmd == "REC_ON") {
    _recoveryBuzzer.setContinuous(true);
    out.println(F("[OK] Recovery buzzer ON (continuous)"));
  }
  else if (cmd == "REC_OFF") {
    _recoveryBuzzer.silence();
    out.println(F("[OK] Recovery buzzer OFF"));
  }
  else if (cmd.startsWith("PATTERN_") && cmd.length() == 9) {
    char c = cmd.charAt(8);
    if (c >= '1' && c <= '9') {
      _uiBuzzer.beepPattern(c - '0');
      out.print(F("[OK] UI buzzer pattern of "));
      out.print(c - '0');
      out.println(F(" beeps"));
    } else {
      out.println(F("[ERR] Pattern count must be 1..9"));
    }
  }

  // ----- Battery monitor commands -----
  else if (cmd == "BATT") {
    float v = _batteryMonitor.readVoltage();
    out.print(F("[BATT] "));
    out.print(v, 3);
    out.print(F(" V ("));
    out.print(_batteryMonitor.healthString());
    out.println(F(")"));
  }
  else if (cmd == "BATT_STREAM_ON") {
    _batteryMonitor.setStatsTracking(true);
    out.println(F("[BATT] Stats tracking ON. Min/Max/Avg will be recorded."));
    out.println(F("       Use BATT_STATS to print, BATT_STREAM_OFF to stop."));
  }
  else if (cmd == "BATT_STREAM_OFF") {
    _batteryMonitor.setStatsTracking(false);
    out.println(F("[BATT] Stats tracking OFF."));
  }
  else if (cmd == "BATT_STATS") {
    if (!_batteryMonitor.isStatsTracking() && _batteryMonitor.sampleCount() == 0) {
      out.println(F("[BATT] No stats captured. Run BATT_STREAM_ON first."));
      return;
    }
    out.print(F("[BATT] Samples: "));
    out.println(_batteryMonitor.sampleCount());
    out.print(F("       Min: "));
    out.print(_batteryMonitor.minVoltage(), 3);
    out.println(F(" V"));
    out.print(F("       Avg: "));
    out.print(_batteryMonitor.avgVoltage(), 3);
    out.println(F(" V"));
    out.print(F("       Max: "));
    out.print(_batteryMonitor.maxVoltage(), 3);
    out.println(F(" V"));
    out.print(F("       Sag: "));
    out.print(_batteryMonitor.maxVoltage() - _batteryMonitor.minVoltage(), 3);
    out.println(F(" V"));
  }
  else if (cmd == "BATT_RESET") {
    _batteryMonitor.resetStats();
    out.println(F("[BATT] Stats cleared."));
  }

  // ----- Pyro disarm / estop -----
  else if (cmd == "DISARM") {
    disarmPyro(out, false);
  }
  else if (cmd == "ESTOP") {
    emergencyStop(out);
  }

  // ----- Flight arming -----
  else if (cmd == "ARM") {
    _flightArmed = true;
    out.println(F("[ARMED] Flight-armed flag set."));
    out.println(F("        Disconnect umbilical to enter autonomous flight mode."));
  }
  else if (cmd == "DISARM_FLIGHT") {
    _flightArmed = false;
    out.println(F("[OK] Flight-armed flag cleared."));
  }
  else if (cmd == "LOG_START") {
    _logging = true;
    out.println(F("[OK] Data logging started."));
  }
  else if (cmd == "LOG_STOP") {
    _logging = false;
    out.println(F("[OK] Data logging stopped."));
  }

  // ----- Sensor streaming -----
  else if (cmd == "STREAM_SENSORS") {
    _streaming = true;
    _streamingTarget = &out;
    _lastStreamPrintMs = 0;
    out.println(F(">>> STREAMING SENSORS @ 20Hz. Send any command to stop."));
  }
  else if (cmd == "STOP") {
    _streaming = false;
    _streamingTarget = nullptr;
    out.println(F(">>> Stopped."));
  }

  else if (cmd == "FORMAT_FLASH") {
    out.println(F("Formatting flash... (takes ~10 seconds)"));
    if (_flashLogger.formatFlash()) {
      out.println(F("[OK] Flash formatted."));
    } else {
      out.println(F("[ERR] Format failed."));
    }
  }
  else if (cmd == "LOG_LIST") {
    _flashLogger.listLogs(out);
  }
  else if (cmd.startsWith("LOG_DUMP ")) {
    String filename = cmd.substring(9);
    filename.trim();
    _flashLogger.dumpLog(filename.c_str(), out);
  }
  else if (cmd == "LOG_START") {
    const char* fname = _flashLogger.nextSessionFilename();
    if (_flashLogger.startSession(fname, Config::Build::STAGE_NAME)) {
      _logging = true;
      out.print(F("[OK] Logging started: "));
      out.println(fname);
    } else {
      out.println(F("[ERR] Could not start logging."));
    }
  }
  else if (cmd == "LOG_STOP") {
    _flashLogger.endSession();
    _logging = false;
    out.println(F("[OK] Logging stopped."));
  }
  else if (cmd == "LOG_STATUS") {
    out.print(F("Logging: "));
    out.println(_flashLogger.isLogging() ? F("ACTIVE") : F("STOPPED"));
    out.print(F("Free: "));
    out.print((uint32_t)(_flashLogger.bytesFree() / 1024));
    out.println(F(" KB"));
  }

  // ----- Canard bench testing (sustainer only) -----
  else if (cmd == "CANARD_BENCH_ON") {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build. (Booster has no canards.)"));
      return;
    }
    if (_flightArmed) {
      out.println(F("[ERR] Cannot enter bench mode while flight is armed."));
      out.println(F("      Send DISARM_FLIGHT first."));
      return;
    }
    _rateDamper->setBenchSuspend(true);
    out.println(F("[BENCH] Controller SUSPENDED. Raw servo writes enabled."));
    out.println(F("        Use: CANARD_TEST <A|B|C|D> <angle 65..115>"));
    out.println(F("        Use: CANARD_BENCH_OFF to resume normal operation"));
    out.println(F(""));
    out.println(F("        SAFETY REMINDER:"));
    out.println(F("        Pyro wires must be disconnected at PCBA screw terminals."));
    out.println(F("        Rocket must be on bench, fin tabs free to move in open air."));
  }
  else if (cmd == "CANARD_BENCH_OFF") {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build."));
      return;
    }
    _rateDamper->setBenchSuspend(false);
    out.println(F("[OK] Controller resumed. Fins centered."));
  }
  else if (cmd.startsWith("CANARD_TEST ")) {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build."));
      return;
    }
    if (!_rateDamper->isBenchSuspended()) {
      out.println(F("[ERR] Not in bench mode. Send CANARD_BENCH_ON first."));
      return;
    }

    // Parse: "CANARD_TEST X NNN"  where X is A|B|C|D and NNN is within
    // the controller's current mechanical deflect range from 90°.
    const int testLo = (int)(90.0f - RateDampingController::MAX_SERVO_DEFLECT_DEG);
    const int testHi = (int)(90.0f + RateDampingController::MAX_SERVO_DEFLECT_DEG);
    String args = cmd.substring(12);
    args.trim();
    if (args.length() < 3) {
      out.print(F("[ERR] Usage: CANARD_TEST <A|B|C|D> <angle "));
      out.print(testLo);
      out.print(F(".."));
      out.print(testHi);
      out.println(F(">"));
      return;
    }
    char fin = args.charAt(0);
    if (fin < 'A' || fin > 'D') {
      out.println(F("[ERR] First arg must be A, B, C, or D"));
      return;
    }
    String angleStr = args.substring(1);
    angleStr.trim();
    int angle = angleStr.toInt();

    // Safety clamp — the writeRawServoX functions also clamp, but reject
    // out-of-range explicitly here so the user sees what happened.
    // Uses the controller's MAX_SERVO_DEFLECT_DEG constant as source of truth.
    if (angle < testLo || angle > testHi) {
      out.print(F("[ERR] Angle must be "));
      out.print(testLo);
      out.print(F(".."));
      out.print(testHi);
      out.print(F(" (controller mech limit is +/-"));
      out.print((int)RateDampingController::MAX_SERVO_DEFLECT_DEG);
      out.println(F(" deg from 90)"));
      return;
    }

    switch (fin) {
      case 'A': _rateDamper->writeRawServoA(angle); break;
      case 'B': _rateDamper->writeRawServoB(angle); break;
      case 'C': _rateDamper->writeRawServoC(angle); break;
      case 'D': _rateDamper->writeRawServoD(angle); break;
    }
    out.print(F("[OK] Servo "));
    out.print(fin);
    out.print(F(" commanded to "));
    out.print(angle);
    out.println(F(" deg"));
  }

  else if (cmd == "CANARD_TRIM_GET") {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build."));
      return;
    }
    out.print(F("[TRIM] A="));
    out.print(_rateDamper->trimA());
    out.print(F("  B="));
    out.print(_rateDamper->trimB());
    out.print(F("  C="));
    out.print(_rateDamper->trimC());
    out.print(F("  D="));
    out.println(_rateDamper->trimD());
    out.println(F("       (deg, subtracted from commanded angle before write)"));
  }

  else if (cmd.startsWith("CANARD_TRIM_SET ")) {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build."));
      return;
    }
    // Parse: "CANARD_TRIM_SET X NN"  where X is A|B|C|D and NN is -7..7
    String args = cmd.substring(16);
    args.trim();
    if (args.length() < 3) {
      out.println(F("[ERR] Usage: CANARD_TRIM_SET <A|B|C|D> <trim -7..7>"));
      return;
    }
    char fin = args.charAt(0);
    if (fin < 'A' || fin > 'D') {
      out.println(F("[ERR] First arg must be A, B, C, or D"));
      return;
    }
    String trimStr = args.substring(1);
    trimStr.trim();
    int trimVal = trimStr.toInt();

    if (!_rateDamper->setTrimOne(fin, trimVal)) {
      out.print(F("[ERR] Trim must be in range -"));
      out.print(RateDampingController::MAX_TRIM_DEG);
      out.print(F(" to +"));
      out.print(RateDampingController::MAX_TRIM_DEG);
      out.println(F(" deg. If real trim is larger, re-clock the servo horn."));
      return;
    }
    out.print(F("[OK] Servo "));
    out.print(fin);
    out.print(F(" trim set to "));
    out.print(trimVal);
    out.println(F(" deg"));
    out.println(F("     NOTE: Trim is RAM only. Hard-code in main.cpp for persistence."));

    // Immediately re-center fins so user can visually confirm the trim moved
    // the servo to its new neutral. In NEUTRAL_FIN mode, update() will call
    // centerAllFins() on the next cycle anyway, but this gives instant feedback.
    _rateDamper->centerAllFins();
    out.println(F("     Fins re-centered with new trim. Verify visually."));
  }

  else if (cmd == "CANARD_TRIM_CLEAR") {
    if (_rateDamper == nullptr) {
      out.println(F("[ERR] No controller on this build."));
      return;
    }
    _rateDamper->clearTrims();
    out.println(F("[OK] All servo trims cleared to zero."));
  }

  // ----- Tilt estimator bench testing (sustainer only) -----
  else if (cmd == "TILT_STATUS") {
    if (_tiltEst == nullptr) {
      out.println(F("[ERR] No tilt estimator on this build. (Booster doesn't track tilt.)"));
      return;
    }
    out.print(F("[TILT] Current: "));
    out.print(_tiltEst->tiltDeg(), 2);
    out.println(F(" deg off vertical"));

    out.print(F("       Max:     "));
    out.print(_tiltEst->maxTiltDeg(), 2);
    out.println(F(" deg (since last reset)"));

    out.print(F("       Fault:   "));
    out.println(_tiltEst->rotationFault() ? F("LATCHED (>45 deg seen)") : F("clear"));

    out.print(F("       SafeCone: "));
    out.println(_tiltEst->isWithinSafeCone() ? F("YES (would allow ignition)") : F("NO (would block ignition)"));

    out.print(F("       Quaternion: w="));
    out.print(_tiltEst->quaternionW(), 4);
    out.print(F(" x="));
    out.print(_tiltEst->quaternionX(), 4);
    out.print(F(" y="));
    out.print(_tiltEst->quaternionY(), 4);
    out.print(F(" z="));
    out.println(_tiltEst->quaternionZ(), 4);
  }
  else if (cmd == "TILT_RESET") {
    if (_tiltEst == nullptr) {
      out.println(F("[ERR] No tilt estimator on this build."));
      return;
    }
    if (_flightArmed) {
      out.println(F("[ERR] Cannot reset tilt while flight is armed."));
      out.println(F("      Send DISARM_FLIGHT first."));
      return;
    }
    _tiltEst->reset();
    out.println(F("[TILT] Estimator reset. Rocket is now defined as vertical."));
    out.println(F("       Quaternion = identity. Fault flag cleared."));
  }
  else if (cmd == "TILT_STREAM_ON") {
    if (_tiltEst == nullptr) {
      out.println(F("[ERR] No tilt estimator on this build."));
      return;
    }
    _tiltStreaming = true;
    _tiltStreamingTarget = &out;
    _lastTiltStreamMs = millis();
    out.println(F("[TILT] Streaming ON at 5 Hz. Send TILT_STREAM_OFF to stop."));
  }
  else if (cmd == "TILT_STREAM_OFF") {
    if (_tiltEst == nullptr) {
      out.println(F("[ERR] No tilt estimator on this build."));
      return;
    }
    _tiltStreaming = false;
    _tiltStreamingTarget = nullptr;
    out.println(F("[TILT] Streaming OFF."));
  }
  
  // ----- Pyro arm/fire (parsed last, since they have parameters) -----
  else {
    int8_t idx = parsePyroIndex(cmd, "ARM_PYRO_");
    if (idx >= 0) {
      disarmPyro(out, true);

      if (!_pyros[idx]->hasContinuity()) {
        out.print(F("[ERR] Cannot arm PYRO"));
        out.print(idx);
        out.println(F(" -- no continuity sensed."));
        out.println(F("      Check: (1) FingerTech is ON, (2) load is connected."));
        return;
      }

      _armedPyroIndex = idx;
      _armedPyroAtMs  = millis();
      _pyros[idx]->setArmed(true);
      _uiBuzzer.beepPattern(2, 80, 80, 0);
      out.print(F("[ARMED] PYRO"));
      out.print(idx);
      out.print(F(" -- send FIRE_PYRO_"));
      out.print(idx);
      out.println(F(" within 5 seconds"));
      return;
    }

    idx = parsePyroIndex(cmd, "FIRE_PYRO_");
    if (idx >= 0) {
      if (_armedPyroIndex != idx) {
        out.print(F("[ERR] PYRO"));
        out.print(idx);
        out.println(F(" not armed. Send ARM_PYRO_n first."));
        return;
      }
      if (millis() - _armedPyroAtMs > Config::Pyro::ARM_TIMEOUT_MS) {
        out.println(F("[ERR] Arm window expired. Re-arm and try again."));
        disarmPyro(out, true);
        return;
      }
      bool ok = _pyros[idx]->fire(Config::Pyro::FIRE_TIME_MS);
      _armedPyroIndex = -1;
      if (ok) {
        out.print(F("[FIRING] PYRO"));
        out.print(idx);
        out.print(F(" for "));
        out.print(Config::Pyro::FIRE_TIME_MS);
        out.println(F(" ms"));
      } else {
        out.println(F("[ERR] Fire refused -- continuity lost or already firing"));
      }
      return;
    }

    out.println(F("[ERR] Unknown command. Send HELP."));
  }
}