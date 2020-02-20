#pragma once

#include <string>
#include <vector>

#include "driver/common.h"

namespace drake_mirobot {
namespace driver {

enum class MirobotRobotStatus {
  kRunning = 1,
  kIdle = 2,
};

enum class MirobotDriverStatus {
  kUnready = 1,  //< Not initialized or no comms to arm.
  kLocked = 2,   //< Arm has not yet homed and so may be axis-locked.
  kIdle = 3,     //< Driver is ready to command the arm.
};

struct MirobotState {
  double timestamp;  //< Latest measurement time.

  std::vector<int> numbered_vars;
  JointspaceCoordinate joints_pos;
  CartesianCoordinate end_effector_pose;

  MirobotRobotStatus robot_status;
  MirobotDriverStatus driver_status;
};

class MirobotDriver {
  explicit MirobotDriver(std::string device_name_substring);
  std::string GetSdf();
  MirobotState GetLastMeasuredState();
  void RefreshState();
  void StartMoveTo(JointspaceCoordinate);
  void StartMoveTo(CartesianCoordinate, CartesianMotionMode);
  void StartMoveBy(JointspaceCoordinate);
  void StartMoveBy(CartesianCoordinate, CartesianMotionMode);
  void WaitUntilIdle();
};

}  // namespace driver
}  // namespace drake_mirobot
