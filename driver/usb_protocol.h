#pragma once

#include <string>

#include <fmt/format.h>

#include <drake/common/drake_assert.h>

/// @file Raw transcription of Mirobot pseudo-gcode serial protocol.

namespace drake_mirobot {
namespace driver {

struct JointspaceCoordinate {
  // Relative to 0 = home position (position after calibration step)
  float axis_1;  // Base yaw (degrees)
  float axis_2;  // Lower arm pitch (degrees)
  float axis_3;  // Upper arm pitch (degrees)
  float axis_4;  // Upper arm roll (degrees)
  float axis_5;  // End-effector pitch (degrees)  (AKA "Wu Axis")
  float axis_6;  // End-effector yaw (degrees)
};

struct CartesianCoordinate {
  float x;  // Positive toward front, from base axis.  (mm)
  float y;  // Positive toward arm's left, from base axis.  (mm)
  float z;  // Positive up, from top of pedestal.  (mm)
  float roll;  // (degrees)
  float pitch;  // (degrees)
  float yaw;  // (degrees)
};

enum class CartesianMotionMode {
  kFast = 1,  //< All joints move uniformly.
  kInterpolated = 2,  //< Joint motion adjusted so xyzrpy change uniformly.
};

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

static MiscCommand StatusRequest() { return MiscCommand("?"); }
static MiscCommand Unlock() { return MiscCommand("M50"); }
static MiscCommand RapidHome() { return MiscCommand("$H"); }
static MiscCommand SequentialHome() { return MiscCommand("$HH"); }

static MiscCommand SetPumpPwm(int pump_pwm) {
  DRAKE_DEMAND(pump_pwm >= 0);
  DRAKE_DEMAND(pump_pwm <= 1000);  // Implied max from manual
  return MiscCommand(fmt::format("M3S{}", pump_pwm));
}
static MiscCommand SetPumpOpen() { return SetPumpPwm(1000); }
static MiscCommand SetPumpClosed() { return SetPumpPwm(0); }

static MiscCommand SetGripperPwm(int grip_pwm) {
  // Not clear what the units/limit on this value are.  The manual cites a
  // range of 40-65 (?).
  return MiscCommand(fmt::format("M4E{}", grip_pwm));
}
static MiscCommand SetGripperOpen() { return SetGripperPwm(65); }
static MiscCommand SetGripperClosed() { return SetGripperPwm(40); }

enum class Register {
  kSoftLimitOpen = 20,
  kHardLimitOpen = 21,
};

static MiscCommand Assignment(Register reg, int value) {
  return MiscCommand(fmt::format("${}={}", static_cast<int>(reg), value));
}

}  // namespace driver
}  // namespace drake_mirobot
