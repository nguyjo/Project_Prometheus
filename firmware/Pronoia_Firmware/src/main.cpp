#include "Config.h"
#include "Arduino.h"
#include "IMU.h"
#include "HighGAccel.h"
#include "AccelerationSource.h"
#include "Barometer.h"
#include "StateEstimator.h"
#include "TiltEstimator.h"
#include "PyroChannel.h"
#include "Buzzer.h"
#include "CommandProcessor.h"
#include "CommandLink.h"
#include "FlashLogger.h"
#include "LogRecordBuilder.h"
#include "BatteryMonitor.h"
 
#if defined(BUILD_FOR_BOOSTER)
  #include "BoosterFlightStrategy.h"
#elif defined(BUILD_FOR_SUSTAINER)
  #include "SustainerFlightStrategy.h"
  #include "RateDampingController.h"
#endif
 
// Sensor objects
IMU         imu(Config::Pins::IMU_CS);
HighGAccel  highG(Config::Pins::HIGHG_CS);
AccelerationSource accelSrc(imu, highG);
Barometer baro(Config::Pins::BARO_CS);
StateEstimator estimator(accelSrc, baro);
 
// Pyro channels — one per physical pyro output
PyroChannel pyro0(Config::Pins::PYRO0_FIRE, Config::Pins::PYRO0_CONT, "PYRO0");
PyroChannel pyro1(Config::Pins::PYRO1_FIRE, Config::Pins::PYRO1_CONT, "PYRO1");
PyroChannel pyro2(Config::Pins::PYRO2_FIRE, Config::Pins::PYRO2_CONT, "PYRO2");
PyroChannel pyro3(Config::Pins::PYRO3_FIRE, Config::Pins::PYRO3_CONT, "PYRO3");
 
// Buzzers
Buzzer uiBuzzer(Config::Pins::UI_BUZZER, "UI");
Buzzer recoveryBuzzer(Config::Pins::RECOVERY_BUZZER, "RECOVERY");
 
// Battery monitor
BatteryMonitor batteryMonitor(Config::Pins::VBATT_MONITOR_PIN);

FlashLogger flashLogger;
 
// ===== Sustainer-only subsystems =====
#if defined(BUILD_FOR_SUSTAINER)
  // TiltEstimator — used by the sustainer for the ignition tilt-check interlock.
  TiltEstimator tiltEstimator(imu);
 
  // RateDampingController — active stabilization via 4 canard servos.
  // Lifecycle: instantiated here, begin() called in setup() AFTER servo power
  // is stable, enableControl()/disableControl() managed by SustainerFlightStrategy
  // at the appropriate state transitions.
  RateDampingController rateDamper(
      imu, baro, estimator,
      Config::Pins::CANARD_SERVO_A,
      Config::Pins::CANARD_SERVO_B,
      Config::Pins::CANARD_SERVO_C,
      Config::Pins::CANARD_SERVO_D);
#endif
 
#if defined(BUILD_FOR_SUSTAINER)
CommandProcessor cmdProc(pyro0, pyro1, pyro2, pyro3,
                         uiBuzzer, recoveryBuzzer,
                         imu, highG, accelSrc, baro,
                         flashLogger,
                         batteryMonitor, &rateDamper, &tiltEstimator);
#else
CommandProcessor cmdProc(pyro0, pyro1, pyro2, pyro3,
                         uiBuzzer, recoveryBuzzer,
                         imu, highG, accelSrc, baro,
                         flashLogger, batteryMonitor);   // booster build, no controller
#endif
 
// USB link — no detect pin, always active when USB is connected.
CommandLink usbLink(Serial, cmdProc);
 
// Umbilical link — gated by the GX-12 detect pin.
CommandLink umbilicalLink(Serial4, cmdProc, Config::Pins::EXTERNAL_CONN_DETECT);
 
// ----- Flight strategy (stage-specific) -----
#if defined(BUILD_FOR_BOOSTER)
  BoosterFlightStrategy flightStrategy(
      estimator, accelSrc, baro,
      pyro0,            // PYRO0 = main parachute charge
      uiBuzzer, recoveryBuzzer);
#elif defined(BUILD_FOR_SUSTAINER)
  SustainerFlightStrategy flightStrategy(
      estimator, accelSrc, baro, imu, tiltEstimator,
      pyro0,                  // PYRO0 = sustainer ignitor
      pyro1,                  // PYRO1 = main parachute charge
      uiBuzzer, recoveryBuzzer,
      rateDamper);            // active stabilization controller
#endif
 
// LogRecordBuilder
LogRecordBuilder logBuilder(imu, highG, accelSrc, baro, estimator,
                            #if defined(BUILD_FOR_SUSTAINER)
                              &tiltEstimator,
                            #else
                              nullptr,
                            #endif
                            pyro0, pyro1, pyro2, pyro3);
bool inFlightMode = false;
 
void setup() {
  pyro0.begin();
  pyro1.begin();
  pyro2.begin();
  pyro3.begin();

  batteryMonitor.begin();

  uiBuzzer.begin();
  recoveryBuzzer.begin();

  SPI.begin();   // SPI bus 0 — for IMU and High-G accel; ICM-42688-PC, KX134-1211
  SPI1.begin();  // SPI bus 1 — for Barometer; MS5611-01BA03

  // ===== Serial interfaces — INITIALIZE BEFORE FIRST USE =====
  Serial.begin(115200);
  Serial4.begin(115200);

  uint32_t bootStart = millis();
  while (!Serial && (millis() - bootStart) < 5000) {
    delay(10);
  }

  // Give USB a beat to fully settle after the host connects
  delay(100);

  // Print banner to whichever interface(s) are connected.
  // Skip the interface if no host is present to avoid potential blocking.
  auto printBanner = [](Stream& s) {
    s.println();
    s.println("==========================================");
    s.println("  PROMETHEUS AVIONICS");
    s.print("  Stage:  ");
    s.println(Config::Build::STAGE_NAME);
    s.print("  Build:  ");
    s.print(Config::Build::BUILD_DATE);
    s.print(" ");
    s.println(Config::Build::BUILD_TIME);
    s.println("==========================================");
    s.println("Type HELP for command list.");
    s.println();
  };

  if (Serial) { printBanner(Serial);}
  if (umbilicalLink.isActive()) {printBanner(Serial4);}

  // Initialize sensors
  if (!imu.begin()) {
    if (Serial) Serial.println("IMU init FAILED");
    if (umbilicalLink.isActive()) Serial4.println("IMU init FAILED");
  }
  if (!highG.begin()) {
    if (Serial) Serial.println("HighG init FAILED");
    if (umbilicalLink.isActive()) Serial4.println("HighG init FAILED");
  }
  if (!baro.begin()) {
    if (Serial) Serial.println("Baro init FAILED");
    if (umbilicalLink.isActive()) Serial4.println("Baro init FAILED");
  }

  // ===== Auto-calibration for autonomous boot (booster has no CLI) =====
  // Booster: no umbilical, no CLI. Must self-calibrate at power-up.
  // Sustainer: also calibrates here as a safety net; the operator can re-run
  // CAL_ALL via umbilical before launch to refresh against the rail orientation.
  //
  // Sequence: gyro bias (~2.5s) → barometer ground ref (~1s) → tilt reset.
  // Total ~4 seconds. Rocket MUST be stationary and vertical during this window.
  delay(500);  // let sensors stabilize before sampling

  imu.calibrateGyro(500);

  if (!baro.calibrateBaro(50)) {
    if (Serial) Serial.println("Baro CAL FAILED");
  }

  #if defined(BUILD_FOR_SUSTAINER)
    tiltEstimator.reset();
  #endif

  if (!flashLogger.begin()) {
    if (Serial) Serial.println("Flash init failed - LOG_FORMAT may help");
  }

  // ===== Sustainer-only: bring up the rate-damping controller =====
  // begin() attaches all 4 servo objects to their pins and commands neutral (90°).
  // Servo power MUST be stable at this point. If the rail brown-outs during
  // the first PWM pulses, the MEX-12 internal controllers can latch into a
  // weird state until power-cycled.
  #if defined(BUILD_FOR_SUSTAINER)
      rateDamper.begin();
      // TODO: Change these for rocket rateDamper.setTrim(0, 4, 0, -2);  // A, B, C, D from launch day measurement
    #endif

    // Boot complete signal. Different patterns per stage so the operator can
    // identify which Teensy just powered up:
    //   - Sustainer: one long beep (~600 ms)
    //   - Booster: three short beeps (~150 ms each with gaps)
    // Both are audible audio cues that boot completed successfully.
    // delay() is acceptable in setup() since the flight loop hasn't started.
  #if defined(BUILD_FOR_SUSTAINER)
    uiBuzzer.beepOnce(600);
    delay(650);
    uiBuzzer.update();  // ensures pin goes LOW at end of beep
  #elif defined(BUILD_FOR_BOOSTER)
    for (int i = 0; i < 3; i++) {
      uiBuzzer.beepOnce(150);
      delay(160);                   // wait for beep to finish
      uiBuzzer.update();            // turn off pin
      if (i < 2) delay(150);        // gap between beeps (no gap after last)
    }
  #endif
    }

void loop() {
  // Update sensors first (order matters!)
  imu.update();
  highG.update();
  baro.update();
  accelSrc.update();
  estimator.update();
  batteryMonitor.update();

  #if defined(BUILD_FOR_SUSTAINER)
    // TiltEstimator must run every IMU update for accurate quaternion
    // integration. We run it unconditionally (not gated on flight mode)
    // so it's always tracking — even pre-launch, so the operator can
    // verify tilt readings via STREAM_SENSORS or a STATUS command.
    tiltEstimator.update();

    // RateDampingController is also called every loop iteration. It is
    // internally rate-limited to Config::Rates::CONTROL_HZ (100 Hz), so
    // calling it from the 1 kHz main loop just early-returns when not yet
    // time to update. The controller only actually drives servos when
    // SustainerFlightStrategy has enabled it (STAGE2_BOOST entry to
    // POST_APOGEE_DESCENT entry). Outside that window, _controlEnabled
    // is false and update() centers the fins.
    rateDamper.update();
  #endif

  // Pyro fire pulses are time-limited; update() ends the pulse when its
  // configured duration has elapsed.
  pyro0.update();
  pyro1.update();
  pyro2.update();
  pyro3.update();

  // Buzzer state machines (handles pattern timing, single beeps, etc.)
  uiBuzzer.update();
  recoveryBuzzer.update();

  // Command processor housekeeping (pyro arm-timeout expiry, etc.)
  cmdProc.update();

  // Command interfaces — each reads input and services streaming output.
  usbLink.update();
  umbilicalLink.update();

  // ===== Flight mode entry — latched =====
  #if defined(BUILD_FOR_BOOSTER)
    // ----- Booster: auto-arm after settling delay -----
    // The booster has no umbilical and no USB access on the sled, so it cannot
    // be armed via CLI. We auto-enter flight mode 30 seconds after boot.
    //
    // Safety analysis:
    //   - The booster's only pyro is PYRO0 (main parachute charge). A parachute
    //     deploy on the ground is loud but not dangerous.
    //   - The state machine still gates pyro firing on detected boost, coast,
    //     and descent conditions. A stationary booster on the pad cannot satisfy
    //     these conditions, so the pyro will not fire prematurely.
    //   - The 30-second settling delay prevents ground-handling bumps (someone
    //     setting the rocket on the pad) from being misread as liftoff.
    static const uint32_t BOOSTER_AUTO_ARM_DELAY_MS = 30000;
    if (!inFlightMode && (millis() > BOOSTER_AUTO_ARM_DELAY_MS)) {
      inFlightMode = true;
      uiBuzzer.beepOnce(500);  // One long beep: flight mode entered
    }
  #else
    // ----- Sustainer: require explicit ARM + umbilical disconnect -----
    // The sustainer has PYRO0 = motor ignitor, which is safety-critical. We
    // require operator intent (ARM via umbilical CLI) AND physical confirmation
    // of the rocket being ready (umbilical disconnected at the pad).
    if (!inFlightMode &&
        cmdProc.isFlightArmed() &&
        !umbilicalLink.isActive()) {
      inFlightMode = true;
      uiBuzzer.beepOnce(500);  // One long beep: flight mode entered
    }
  #endif
 
  if (inFlightMode) {
    flightStrategy.update();
  }
 
  // 50 Hz logging update
  static uint32_t lastLogMs = 0;
  if (flashLogger.isLogging() && (millis() - lastLogMs >= 20)) {
    lastLogMs = millis();

    // Flight state from the active strategy's state machine.
    // NOTE: The numeric IDs are defined by each strategy's State enum and
    // collide between booster and sustainer builds (e.g. ID 1 = BOOST on
    // booster, but STAGE1_BOOST on sustainer). The post-flight decoder
    // must know which build produced the log to map IDs back to names.
    const uint8_t state = flightStrategy.currentStateId();

#if defined(BUILD_FOR_SUSTAINER)
    // Sustainer: pass the rate damper's actual commanded servo angles.
    // The controller stores commanded angles as floats in [65..115]; cast
    // to int8_t truncates toward zero, giving clean 1-degree resolution
    // and a value that always fits in int8's ±127 range.
    LogRecord rec = logBuilder.build(state, cmdProc.isFlightArmed(),
                                     /*baro_calibrated=*/true,
                                     (int8_t)rateDamper.commandedDegA(),
                                     (int8_t)rateDamper.commandedDegB(),
                                     (int8_t)rateDamper.commandedDegC(),
                                     (int8_t)rateDamper.commandedDegD());
#else
    // Booster: no canard controller. The canard*_deg fields default to 0
    // in LogRecordBuilder::build(), so they'll log as zero throughout.
    LogRecord rec = logBuilder.build(state, cmdProc.isFlightArmed(),
                                     /*baro_calibrated=*/true);
#endif

    flashLogger.logRecord(rec);
  }
}