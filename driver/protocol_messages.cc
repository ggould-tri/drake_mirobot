#include "driver/protocol_messages.h"

#include <iomanip>
#include <ostream>
#include <sstream>

namespace drake_mirobot {
namespace driver {

namespace {
std::string CoordModeString(CoordinateMode coord_mode) {
  switch (coord_mode) {
    case CoordinateMode::kAbsolute: return "G90";
    case CoordinateMode::kRelative: return "G91";
    default: DRAKE_ASSERT(false);
  }
  DRAKE_UNREACHABLE();
}

std::string MotionModeString(CartesianMotionMode motion_mode) {
  switch (motion_mode) {
    case CartesianMotionMode::kFast: return "G0";
    case CartesianMotionMode::kInterpolated: return "G1";
    default: DRAKE_ASSERT(false);
  }
  DRAKE_UNREACHABLE();
}
}  // namespace

JointspaceMotionCommand::JointspaceMotionCommand(
    JointspaceCoordinate coord,
    CoordinateMode coord_mode,
    int max_deg_per_min)
    : coord_(coord),
      coord_mode_(coord_mode),
      max_deg_per_min_(max_deg_per_min) {
  DRAKE_DEMAND(max_deg_per_min > 0);
}

std::string JointspaceMotionCommand::ToString() {
  std::ostringstream stream;
  stream << std::setprecision(3)
         << "M21 " << CoordModeString(coord_mode_);
  if (coord_.axis_1) { stream << " X" << coord_.axis_1; }
  if (coord_.axis_2) { stream << " Y" << coord_.axis_2; }
  if (coord_.axis_3) { stream << " Z" << coord_.axis_3; }
  if (coord_.axis_4) { stream << " A" << coord_.axis_4; }
  if (coord_.axis_5) { stream << " B" << coord_.axis_5; }
  if (coord_.axis_6) { stream << " C" << coord_.axis_6; }
  stream << " F" << max_deg_per_min_;
  return stream.str();
}

CartesianMotionCommand::CartesianMotionCommand(
    CartesianCoordinate coord,
    CoordinateMode coord_mode,
    CartesianMotionMode motion_mode,
    int max_mm_per_min)
    : coord_(coord),
      coord_mode_(coord_mode),
      motion_mode_(motion_mode),
      max_mm_per_min_(max_mm_per_min) {
  DRAKE_DEMAND(max_mm_per_min > 0);
}

std::string CartesianMotionCommand::ToString() {
  std::ostringstream stream;
  stream << std::setprecision(3)
         << "M20 " << CoordModeString(coord_mode_)
         << " " << MotionModeString(motion_mode_);
  if (coord_.x) { stream << " X" << coord_.x; }
  if (coord_.y) { stream << " Y" << coord_.y; }
  if (coord_.z) { stream << " Z" << coord_.z; }
  // The examples in the manual include A, B, C even if they are zero.
  stream << " A" << coord_.roll;
  stream << " B" << coord_.pitch;
  stream << " C" << coord_.yaw;
  stream << " F" << max_mm_per_min_;
  return stream.str();
}

}  // namespace driver
}  // namespace drake_mirobot
