#include "driver/usb_serial_link.h"

#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <sstream>

#include <drake/common/drake_assert.h>

namespace drake_mirobot {
namespace driver {

UsbSerialLink::UsbSerialLink(std::string device_name_substring) {
  glob_t glob_result;
  int glob_ret = glob(("/dev/*" + device_name_substring + "*").c_str(),
                      0, nullptr, &glob_result);
  if(glob_ret != 0) {
    std::ostringstream ss;
    ss << "Unable to find serial device: " << glob_ret;
    throw std::runtime_error(ss.str());
  }
  if (glob_result.gl_pathc != 1) {
    std::ostringstream ss;
    ss << "Wanted exactly one serial device but found " << glob_result.gl_pathc;
    throw std::runtime_error(ss.str());
  }

  device_path_ = glob_result.gl_pathv[0];
  fd_ = open(device_path_.c_str(), O_RDWR);
  DRAKE_DEMAND(fd_ >= 0);

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if(tcgetattr(fd_, &tty) != 0) {
    std::ostringstream ss;
    ss << "Error %i from tcgetattr:" << errno << "(" << strerror(errno) << ")";
    throw std::runtime_error(ss.str());
  }
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // One stop
  tty.c_cflag |= CS8;       // 8 bit
  tty.c_cflag &= ~CRTSCTS;  // No HW flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No SW flow control
  tty.c_cflag |= CLOCAL;    // Local control/HUP
  tty.c_cflag |= CREAD;     // Readable
  tty.c_lflag |= ICANON;    // Canonical (line-buffered) mode
  tty.c_lflag &= ~(ECHO | ECHOE | ECHONL);  // No echo mode
  tty.c_lflag &= ~ISIG;     // No magic signaling characters
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Raw chars
  tty.c_oflag &= ~OPOST;    // No magic output newlines
  tty.c_oflag &= ~ONLCR;    // No DOS-style CRLFs
  tty.c_cc[VTIME] = 1;      // Allow a modest 0.1s block on reads
  tty.c_cc[VMIN] = 0;       // Return even if there is no data
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::ostringstream ss;
    ss << "Error %i from tcsetattr:" << errno << "(" << strerror(errno) << ")";
    throw std::runtime_error(ss.str());
  }
}

UsbSerialLink::~UsbSerialLink() {
  close(fd_);
  fd_ = -1;
}

void UsbSerialLink::WriteLine(std::string line) {
  ssize_t written = 0;
  written = write(fd_, line.c_str(), line.size());
  DRAKE_DEMAND(written > 0);
  written = write(fd_, "\n", 1);
  DRAKE_DEMAND(written > 0);
}

std::string UsbSerialLink::ReadLineBlocking() {
  DRAKE_DEMAND(false);  // not implemented
  DRAKE_UNREACHABLE();
}

std::vector<std::string> UsbSerialLink::ReadLineNonblocking() {
  DRAKE_DEMAND(false);  // not implemented
  DRAKE_UNREACHABLE();
}

bool UsbSerialLink::Eof() {
  DRAKE_DEMAND(false);  // not implemented
  DRAKE_UNREACHABLE();
}

}  // namespace driver
}  // namespace drake_mirobot
