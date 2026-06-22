// =============================================================================
// FlashLogger.cpp
// =============================================================================
#include "FlashLogger.h"

// Use the QSPI-flash variant of LittleFS that PJRC provides.
// This auto-detects the W25Q128JV (16MB) chip on the Teensy 4.1's QSPI socket.
LittleFS_QSPIFlash flashFS;

FlashLogger::FlashLogger() {
}

bool FlashLogger::begin() {
  // Attempt to mount the existing filesystem.
  // If this fails, the chip may be unformatted — caller should run
  // formatFlash() to initialize it.
  if (!flashFS.begin()) {
    return false;
  }
  _ready = true;
  return true;
}

// -----------------------------------------------------------------------------
// Session management
// -----------------------------------------------------------------------------

bool FlashLogger::startSession(const char* filename, const char* stage_name) {
  if (!_ready) return false;
  if (_logging) {
    // Already logging — close the existing session first
    endSession();
  }

  _logFile = flashFS.open(filename, FILE_WRITE);
  if (!_logFile) {
    return false;
  }

  // Write the session header as the first 64 bytes
  SessionHeader hdr = {};
  hdr.magic        = LOG_MAGIC;
  hdr.version      = LOG_VERSION;
  hdr.boot_time_ms = millis();
  // Copy stage_name safely, ensuring null termination
  strncpy(hdr.stage, stage_name, sizeof(hdr.stage) - 1);
  hdr.stage[sizeof(hdr.stage) - 1] = '\0';
  strncpy(hdr.build_date, __DATE__, sizeof(hdr.build_date) - 1);
  hdr.build_date[sizeof(hdr.build_date) - 1] = '\0';
  strncpy(hdr.build_time, __TIME__, sizeof(hdr.build_time) - 1);
  hdr.build_time[sizeof(hdr.build_time) - 1] = '\0';

  size_t written = _logFile.write((const uint8_t*)&hdr, sizeof(hdr));
  if (written != sizeof(hdr)) {
    _logFile.close();
    return false;
  }

  _bufferFill = 0;
  _logging    = true;
  return true;
}

void FlashLogger::endSession() {
  if (!_logging) return;
  flush();
  _logFile.close();
  _logging = false;
}

// -----------------------------------------------------------------------------
// Logging records
// -----------------------------------------------------------------------------

bool FlashLogger::logRecord(const LogRecord& rec) {
  if (!_logging) return false;

  // Check if adding this record would overflow the buffer.
  // If so, flush first.
  if (_bufferFill + sizeof(LogRecord) > WRITE_BUFFER_SIZE) {
    flush();
  }

  // Append the record to the buffer
  memcpy(&_writeBuffer[_bufferFill], &rec, sizeof(LogRecord));
  _bufferFill += sizeof(LogRecord);

  return true;
}

void FlashLogger::flush() {
  if (!_logging || _bufferFill == 0) return;

  size_t written = _logFile.write(_writeBuffer, _bufferFill);
  // If write was partial, we've potentially lost data — but at this point
  // there's not much we can do besides note it. The flash is either full or
  // failed. Continue logging will probably also fail; user will see this
  // in the file size when dumping.
  (void)written;

  // Force LittleFS to commit the data to flash now (not buffered in RAM).
  // This costs a bit of performance but means power loss doesn't lose the
  // last few records.
  _logFile.flush();

  _bufferFill = 0;
}

// -----------------------------------------------------------------------------
// Flash management
// -----------------------------------------------------------------------------

bool FlashLogger::formatFlash() {
  if (_logging) {
    endSession();
  }
  _ready = false;

  // Format the entire chip. This is blocking and takes several seconds.
  if (!flashFS.quickFormat()) {
    return false;
  }

  // Re-mount after format
  if (!flashFS.begin()) {
    return false;
  }

  _ready = true;
  return true;
}

uint64_t FlashLogger::bytesFree() {
  if (!_ready) return 0;
  return flashFS.totalSize() - flashFS.usedSize();
}

uint64_t FlashLogger::bytesUsed() {
  if (!_ready) return 0;
  return flashFS.usedSize();
}

// -----------------------------------------------------------------------------
// Inspection / dump
// -----------------------------------------------------------------------------

void FlashLogger::listLogs(Stream& out) {
  if (!_ready) {
    out.println(F("[ERR] Flash not ready."));
    return;
  }

  out.println(F("---- LOG FILES ----"));
  File root = flashFS.open("/");
  if (!root) {
    out.println(F("[ERR] Could not open root directory."));
    return;
  }

  File entry;
  int count = 0;
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      out.print(F("  "));
      out.print(entry.name());
      out.print(F("  ("));
      out.print(entry.size());
      out.print(F(" bytes, "));
      // Number of records = (size - header) / record size
      if (entry.size() >= sizeof(SessionHeader)) {
        uint32_t numRecords = (entry.size() - sizeof(SessionHeader)) / sizeof(LogRecord);
        out.print(numRecords);
        out.print(F(" records"));
      } else {
        out.print(F("incomplete"));
      }
      out.println(F(")"));
      count++;
    }
    entry.close();
  }
  root.close();

  if (count == 0) {
    out.println(F("  (no files)"));
  }
  out.print(F("Free space: "));
  out.print((uint32_t)(bytesFree() / 1024));
  out.println(F(" KB"));
  out.println(F("-------------------"));
}

void FlashLogger::dumpLog(const char* filename, Stream& out) {
  if (!_ready) {
    out.println(F("[ERR] Flash not ready."));
    return;
  }

  File f = flashFS.open(filename, FILE_READ);
  if (!f) {
    out.print(F("[ERR] File not found: "));
    out.println(filename);
    return;
  }

  out.print(F("---- BEGIN LOG: "));
  out.print(filename);
  out.println(F(" ----"));

  // Read in chunks of one record, hex-encode, write line by line.
  // Format: each line is "REC " followed by hex-encoded bytes, then \n.
  // The first record is the header. Subsequent records are LogRecord.
  // Host script splits on "REC " prefix, decodes hex, interprets structure.
  uint8_t recBuf[sizeof(LogRecord)];
  uint32_t recIdx = 0;
  while (f.available() >= (int)sizeof(LogRecord)) {
    size_t n = f.read(recBuf, sizeof(LogRecord));
    if (n != sizeof(LogRecord)) break;

    out.print(F("REC "));
    for (size_t i = 0; i < sizeof(LogRecord); i++) {
      if (recBuf[i] < 0x10) out.print('0');
      out.print(recBuf[i], HEX);
    }
    out.println();
    recIdx++;

    // Yield occasionally so the rest of the system can breathe.
    // We're dumping hex over USB serial at 115200 baud — slow operation.
    if ((recIdx & 0x3F) == 0) {
      yield();
    }
  }

  f.close();
  out.print(F("---- END LOG: "));
  out.print(recIdx);
  out.println(F(" records ----"));
}

const char* FlashLogger::nextSessionFilename() {
  // Scan existing files and find the next available "flight_NNN.log" name.
  // Simple linear search up to 999 sessions.
  if (!_ready) {
    strncpy(_filenameBuf, "flight_000.log", sizeof(_filenameBuf));
    return _filenameBuf;
  }

  for (int i = 0; i < 1000; i++) {
    snprintf(_filenameBuf, sizeof(_filenameBuf), "flight_%03d.log", i);
    if (!flashFS.exists(_filenameBuf)) {
      return _filenameBuf;
    }
  }

  // Fell through — chip is full of flight logs. Use a generic name.
  strncpy(_filenameBuf, "flight_FULL.log", sizeof(_filenameBuf));
  return _filenameBuf;
}