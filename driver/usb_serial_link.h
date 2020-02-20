#pragma once

#include <string>
#include <vector>

namespace drake_mirobot {
namespace driver {

/// Slightly higher level interface to line-oriented USB serial protocols of
/// the sort mirobot uses.  Will drop data in preference to blocking, because
/// anything going over USB is an unreliable protocol anyway.
class UsbSerialLink {
 public:
  /// Given a unique substring of a USB serial device name, construct a
  /// UsbSerialLink on that device.
  explicit UsbSerialLink(std::string device_name_substring);
  ~UsbSerialLink();

  /// Writes a line, or discards it if it would block.  Newline not required.
  void WriteLine(std::string line);

  /// Reads a line, blocking until one is available.  Newline is discarded.
  /// If the stream EOFs while this is blocking, returns an empty string (as
  /// if the stream had ended with an extra newline).
  std::string ReadLineBlocking();

  /// Reads some lines iff it would not block for long.  Returns empty if no
  /// lines were available.  Newline characters are discarded.
  std::vector<std::string> ReadLineNonblocking();

  /// True iff the remote end has disconnected.
  bool Eof();

 private:
  std::string device_path_;
  int fd_;
  std::string received_;  // An unnecessarily large buffer.
};

}  // namespace driver
}  // namespace drake_mirobot
