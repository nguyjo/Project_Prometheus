// =============================================================================
// FlashLogger.h
// =============================================================================
// Binary flight data logger backed by LittleFS on QSPI flash (W25Q128JV, 16MB).
//
// DESIGN GOALS:
//   - Log at 50 Hz throughout flight (sufficient for 1g resolution events)
//   - Single 64-byte record format with all critical + analysis fields
//   - LittleFS filesystem so multiple flights can coexist on the chip
//   - Pre-flight erase ensures no in-flight block-erase pauses
//   - Dump command for post-flight readout over serial
//
// USAGE:
//   1. Call begin() once at startup. Mounts LittleFS, verifies flash health.
//   2. Pre-flight (one-time per flight): send the FORMAT_FLASH command via CLI
//      to erase the chip and prepare a fresh logging session. This ensures no
//      erase operations during flight.
//   3. Call startSession(filename) to open a new log file for the flight.
//   4. In flight loop, call logRecord(...) at the desired rate. Internally
//      buffered — actual writes happen during flush() or when buffer fills.
//   5. Call endSession() at touchdown to close the file cleanly.
//   6. Post-flight, use the LOG_LIST and LOG_DUMP CLI commands to retrieve
//      logs over USB.
//
// PERFORMANCE NOTES:
//   - 64-byte records at 50 Hz = 3.2 KB/sec sustained writes
//   - LittleFS sustained write speed ~100+ KB/sec on Teensy 4.1, well above
//     our requirement
//   - LittleFS handles wear leveling and block erasure automatically
//   - Buffered writes (256-byte chunks) reduce flash wear and improve speed
//
// RECORD FORMAT (64 bytes packed):
//   See LogRecord struct definition below. All multi-byte values are stored
//   in the Teensy's native endian (little-endian on ARM Cortex-M7).
// =============================================================================

#ifndef FLASHLOGGER_H
#define FLASHLOGGER_H

#include <Arduino.h>
#include <LittleFS.h>

// =============================================================================
// Binary record format
// =============================================================================
// __attribute__((packed)) ensures no compiler-inserted padding between fields.
// Total size: exactly 64 bytes. Verified at compile time via static_assert.
//
// Field ordering chosen so 4-byte floats are naturally aligned where possible.
// Small fields (bytes, bits) clustered together to maximize packing.
struct __attribute__((packed)) LogRecord {
    // ===== Timing and state (8 bytes) =====
    uint32_t timestamp_us;       // micros() at record creation
    uint8_t  flight_state;       // FlightStrategy enum value
    uint8_t  pyro_states;        // 4 channels, 2 bits each: 0=idle, 1=armed, 2=firing, 3=spent
    uint8_t  pyro_continuity;    // 4 channels, 1 bit each (LSB = pyro 0)
    uint8_t  flags;              // bit 0: rotation_fault
                                 // bit 1: using_highg (KX134 active vs IMU)
                                 // bit 2: flight_armed
                                 // bit 3: baro_calibrated
                                 // bits 4-7: reserved

    // ===== Primary flight state (16 bytes) =====
    float    altitude_m;         // Fused altitude from StateEstimator
    float    velocity_mps;       // Fused velocity (positive = ascending)
    float    accel_axial_mps2;   // Gravity-removed axial acceleration
    float    tilt_deg;           // Off-vertical angle from TiltEstimator

    // ===== Rotation rates (12 bytes) =====
    float    gyro_x_dps;         // Pitch rate (rotation about Lat-1)
    float    gyro_y_dps;         // Roll rate (rotation about axial)
    float    gyro_z_dps;         // Yaw rate (rotation about Lat-2)

    // ===== Raw sensor readings (16 bytes) =====
    float    baro_altitude_m;    // Raw barometric altitude (pre-fusion)
    float    imu_accel_g;        // IMU axial acceleration in g's
    float    highg_accel_g;      // KX134 axial acceleration in g's
    float    baro_pressure_mbar; // Atmospheric pressure

    // ===== Lateral accels for tumbling detection (8 bytes) =====
    float    imu_lat1_g;         // Lateral 1 (Chip +X)
    float    imu_lat2_g;         // Lateral 2 (Chip +Z)

    // ===== Servo positions for sustainer rate damping (4 bytes) =====
    // Packed as 4 signed int8 values, each representing servo angle in
    // degrees. Range -127..+127 fits comfortably in MEX-12's ±60° range
    // and gives 1° resolution.
    int8_t   canard1_deg;
    int8_t   canard2_deg;
    int8_t   canard3_deg;
    int8_t   canard4_deg;
};

static_assert(sizeof(LogRecord) == 64, "LogRecord must be exactly 64 bytes");

// =============================================================================
// Session header — written at the start of each log file
// =============================================================================
// 64 bytes, same size as a record. Identified by the LOG_MAGIC value in the
// first 4 bytes (no LogRecord can have this value in its timestamp field since
// the value is unrealistic for micros()).
struct __attribute__((packed)) SessionHeader {
    uint32_t magic;              // 4
    uint32_t version;            // 4
    uint32_t boot_time_ms;       // 4
    char     stage[8];           // 8
    char     build_date[12];     // 12
    char     build_time[9];      // 9
    uint8_t  reserved[23];       // 23 (padding to make total size 64 bytes)
};

static_assert(sizeof(SessionHeader) == 64, "SessionHeader must be exactly 64 bytes");

constexpr uint32_t LOG_MAGIC   = 0x474F4C46;  // 'FLOG' in ASCII (little-endian)
constexpr uint32_t LOG_VERSION = 1;

// =============================================================================
// FlashLogger class
// =============================================================================
class FlashLogger {
  public:
    FlashLogger();

    // Initialize the QSPI flash and mount LittleFS.
    // Returns false if hardware init failed.
    bool begin();

    // True if begin() succeeded.
    bool isReady() const { return _ready; }

    // ===== Session lifecycle =====
    // Open a new log file for writing. filename should be short (≤ 32 chars).
    // Convention: "flight_NNN.log" where NNN is a sequence number.
    // Returns false if file couldn't be opened (e.g., flash full).
    bool startSession(const char* filename, const char* stage_name);

    // Close the current log file. Flushes any buffered records first.
    void endSession();

    // True if a session is currently open for logging.
    bool isLogging() const { return _logging; }

    // ===== Per-record logging =====
    // Append a single record to the open log file. Internally buffered.
    // Returns false if no session is open or write failed.
    bool logRecord(const LogRecord& rec);

    // Force any buffered records out to flash immediately. Called by
    // endSession(); also useful before known-slow operations to keep buffer
    // small and predictable.
    void flush();

    // ===== Flash management =====
    // Erase entire flash chip and re-initialize LittleFS. Call PRE-FLIGHT
    // to guarantee no in-flight erase operations.
    // BLOCKING — takes several seconds.
    bool formatFlash();

    // ===== Inspection / dump =====
    // List all log files on the chip to the given Stream.
    void listLogs(Stream& out);

    // Dump a specific log file to the given Stream as hex-encoded binary.
    // Each record on its own line. Use a script on the host to decode.
    void dumpLog(const char* filename, Stream& out);

    // Free space in bytes on the LittleFS partition.
    uint64_t bytesFree();
    uint64_t bytesUsed();

    // Suggest a filename for the next session based on existing files.
    // Returns a static buffer, valid until next call.
    const char* nextSessionFilename();

  private:
    bool _ready   = false;
    bool _logging = false;
    File _logFile;

    // Internal write buffer — accumulates records before flushing to flash.
    // 256 bytes = 4 records, aligned with LittleFS page size.
    static constexpr size_t WRITE_BUFFER_SIZE = 256;
    uint8_t _writeBuffer[WRITE_BUFFER_SIZE];
    size_t  _bufferFill = 0;

    // For nextSessionFilename()
    char _filenameBuf[32];
};

#endif