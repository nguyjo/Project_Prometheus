// =============================================================================
// CommandLink.h
// =============================================================================
// Unified command interface — replaces BenchTool and ExternalConn.
//
// A CommandLink wraps a single Stream (USB Serial or Serial4) and routes
// commands to the shared CommandProcessor. Optionally takes a "detect pin"
// for physical gating — when the detect pin is non-zero and reads HIGH,
// input is ignored. This is how the GX-12 umbilical interface stays dead
// when the umbilical is unplugged.
//
// USAGE in main.cpp:
//
//   // USB interface — no detect pin
//   CommandLink usbLink(Serial, cmdProc);
//
//   // Umbilical interface — gated by GX-12 detect pin
//   CommandLink umbilicalLink(Serial4, cmdProc, Config::Pins::EXTERNAL_CONN_DETECT);
//
//   // In loop():
//   usbLink.update();
//   umbilicalLink.update();
//
// DESIGN NOTES:
//   Before this refactor, BenchTool and ExternalConn were two separate classes
//   that did essentially the same thing — read lines from a Stream and hand
//   them to CommandProcessor. The only real difference was that ExternalConn
//   gated on a physical detect pin. The split also enforced an artificial
//   "caller-aware" distinction in CommandProcessor that let certain commands
//   work only on USB (e.g. STREAM_SENSORS) or only on the umbilical (e.g. ARM).
//
//   That distinction turned out to be more architectural ceremony than safety.
//   The two-step pyro arming protocol prevents accidental pyro fires regardless
//   of interface. The flight-state-machine entry is gated by the physical state
//   of the umbilical (isUmbilicalConnected()), not by which interface sent the
//   ARM command. Sensor streaming over the umbilical is genuinely useful for
//   last-minute pre-flight diagnostics. So we collapsed the two classes into
//   one, and every command works on both interfaces.
//
//   The detect-pin gating remains. It's the only meaningful difference between
//   the interfaces, and it's a physical safety property: when the umbilical is
//   physically unplugged, no commands can arrive over its Stream regardless of
//   what bits might be on the wire.
// =============================================================================

#ifndef COMMANDLINK_H
#define COMMANDLINK_H

#include <Arduino.h>
#include "CommandProcessor.h"

class CommandLink {
  public:
    // Construct a CommandLink for the given Stream. If detectPin is -1, the
    // link is always active (use this for USB). Otherwise, the link is only
    // active when the detect pin reads LOW (use this for the umbilical, where
    // the GX-12 detect pin is pulled to GND when the umbilical is plugged in).
    CommandLink(Stream& stream, CommandProcessor& cmdProc, int detectPin = -1);

    // Configures the detect pin (if any) and reserves the input buffer.
    // Stream.begin() must be called separately — typically in main setup().
    void begin();

    // Call once per loop iteration. Reads available characters, accumulates
    // into a line buffer, dispatches complete lines to the command processor,
    // and services any active sensor streaming back to this stream.
    void update();

    // Returns true if the interface is currently active. For USB (no detect
    // pin), this is always true. For the umbilical, true when the detect pin
    // reads LOW (umbilical plugged in).
    bool isActive() const;

  private:
    Stream&           _stream;
    CommandProcessor& _cmdProc;
    int               _detectPin;   // -1 means "no gating, always active"
    String            _inputBuf;

    static constexpr size_t MAX_LINE_LEN = 64;
};

#endif