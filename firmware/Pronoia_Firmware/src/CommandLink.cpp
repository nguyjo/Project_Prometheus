// =============================================================================
// CommandLink.cpp
// =============================================================================
#include "CommandLink.h"

CommandLink::CommandLink(Stream& stream, CommandProcessor& cmdProc, int detectPin)
  : _stream(stream), _cmdProc(cmdProc), _detectPin(detectPin) {
}

void CommandLink::begin() {
  // Configure the detect pin if one was specified.
  // The pin is pulled HIGH internally; the umbilical bridges it to GND when
  // plugged in, so reading LOW indicates "connected."
  if (_detectPin >= 0) {
    pinMode(_detectPin, INPUT_PULLUP);
  }
  _inputBuf.reserve(MAX_LINE_LEN);
}

bool CommandLink::isActive() const {
  // No detect pin = always active (USB case).
  if (_detectPin < 0) return true;
  // Detect pin specified = active only when pin reads LOW (umbilical plugged in).
  return (digitalRead(_detectPin) == LOW);
}

// void CommandLink::update() {
//   // If this link has a detect pin and the umbilical is unplugged, drain any
//   // stale data in the Stream's input buffer and exit. This prevents partial
//   // lines left over from a previous session from being processed when the
//   // umbilical is replugged.
//   if (!isActive()) {
//     while (_stream.available()) _stream.read();
//     _inputBuf = "";
//     return;
//   }

//   // Service any active sensor streaming on THIS stream. The command processor
//   // tracks which Stream is currently streaming; calling serviceStreaming with
//   // a Stream that isn't the current streaming target is a no-op.
//   _cmdProc.serviceStreaming(_stream);

//   // Read one line of input non-blockingly. Accumulate into a buffer until we
//   // see a newline or carriage return, then dispatch the complete line.
//   while (_stream.available()) {
//     char c = _stream.read();
//     if (c == '\n' || c == '\r') {
//       if (_inputBuf.length() > 0) {
//         _inputBuf.trim();
//         _inputBuf.toUpperCase();
//         _cmdProc.process(_inputBuf, _stream);
//         _inputBuf = "";
//       }
//     } else if (_inputBuf.length() < MAX_LINE_LEN) {
//       _inputBuf += c;
//     }
//     // If buffer is full and no newline yet, characters are silently dropped.
//     // This is intentional — a stuck buffer is preferable to an unbounded one.
//   }
// }

void CommandLink::update() {
  if (!isActive()) {
    while (_stream.available()) _stream.read();
    _inputBuf = "";
    return;
  }

  _cmdProc.serviceStreaming(_stream);

  while (_stream.available()) {
    char c = _stream.read();
    if (c == '\n' || c == '\r') {
      if (_inputBuf.length() > 0) {
        _inputBuf.trim();
        _inputBuf.toUpperCase();
        _cmdProc.process(_inputBuf, _stream);
        _inputBuf = "";
      }
    } else if (_inputBuf.length() < MAX_LINE_LEN) {
      _inputBuf += c;
    }
  }
}