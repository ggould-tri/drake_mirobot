#pragma once

#include <optional>
#include <string>

namespace drake_mirobot {
namespace driver {

/// Slightly higher level interface to line-orient USB serial protocol of the
/// sort mirobot uses.  Will drop data in preference to blocking.
class UsbSerialLink {
 public:
  /// If there is exactly one USB serial device with this substring it its
  /// name, construct a UsbSerialLink on that device.  Otherwise assert.
  UsbSerialLink(std::string device_name_substring);

  /// Writes a line, or discards it if it would block.  Newline not required.
  void WriteLine(std::string line);

  /// Reads a line, blocking until one is available.  Newline is discarded.
  std::string ReadLine();

  /// Reads a line iff it would not block.  Newline is discarded.
  std::optional<std::string> ReadLineBlocking();

  /// True iff the remote end has disconnected.
  bool Eof();
};

}  // namespace driver
}  // namespace drake_mirobot
