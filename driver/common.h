#pragma once

/// @file Structures common between multiple layers of the driver.

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

}  // namespace driver
}  // namespace drake_mirobot
