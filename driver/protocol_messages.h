#pragma once

#include <string>

#include <fmt/format.h>

#include <drake/common/drake_assert.h>

#include "driver/common.h"

/// @file Raw transcription of Mirobot pseudo-gcode serial protocol.

namespace drake_mirobot {
namespace driver {

enum class CoordinateMode {
  kAbsolute = 1,
  kRelative = 2,
};

class Command {
 public:
  virtual std::string ToString()=0;
};

class JointspaceMotionCommand : public Command {
 public:
  JointspaceMotionCommand(JointspaceCoordinate, CoordinateMode,
                          int max_deg_per_min);
  std::string ToString() override;

 private:
  JointspaceCoordinate coord_;
  CoordinateMode coord_mode_;
  int max_deg_per_min_;
};

class CartesianMotionCommand : public Command {
 public:
  CartesianMotionCommand(CartesianCoordinate,
                         CoordinateMode, CartesianMotionMode,
                         int max_mm_per_min);
  std::string ToString() override;

 private:
  CartesianCoordinate coord_;
  CoordinateMode coord_mode_;
  CartesianMotionMode motion_mode_;
  int max_mm_per_min_;
};

class MiscCommand : public Command {
 public:
  MiscCommand(std::string command) : command_(command) {}
  std::string ToString() override { return command_; }
 private:
  std::string command_;
};

MiscCommand StatusRequest() { return MiscCommand("?"); }
MiscCommand Unlock() { return MiscCommand("M50"); }
MiscCommand RapidHome() { return MiscCommand("$H"); }
MiscCommand SequentialHome() { return MiscCommand("$HH"); }

MiscCommand SetPumpPwm(int pump_pwm) {
  DRAKE_DEMAND(pump_pwm >= 0);
  DRAKE_DEMAND(pump_pwm <= 1000);  // Implied max from manual
  return MiscCommand(fmt::format("M3S{}", pump_pwm));
}
MiscCommand SetPumpOpen() { return SetPumpPwm(1000); }
MiscCommand SetPumpClosed() { return SetPumpPwm(0); }

MiscCommand SetGripperPwm(int grip_pwm) {
  // Not clear what the units/limit on this value are.  The manual cites a
  // range of 40-65 (?).
  return MiscCommand(fmt::format("M4E{}", grip_pwm));
}
MiscCommand SetGripperOpen() { return SetGripperPwm(65); }
MiscCommand SetGripperClosed() { return SetGripperPwm(40); }

enum class Register {
  kSoftLimitOpen = 20,
  kHardLimitOpen = 21,
};

MiscCommand Assignment(Register reg, int value) {
  return MiscCommand(fmt::format("${}={}", static_cast<int>(reg), value));
}

}  // namespace driver
}  // namespace drake_mirobot
