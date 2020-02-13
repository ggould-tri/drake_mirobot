#include "driver/usb_protocol.h"

#include <gtest/gtest.h>

namespace drake_mirobot {
namespace driver {

// Test each example from the manual (as of 2020-02-13):
//   https://cdn.yun.sooce.cn/1/9925/pdf/15814209180060c7882c31c3af267.pdf
GTEST_TEST(UsbProtocolTest, CheckCommandExamplesFromManual) {
  EXPECT_EQ(StatusRequest().ToString(), "?");  // Section 2.1
  EXPECT_EQ(Unlock().ToString(), "M50");       // Section 2.2
  EXPECT_EQ(Assignment(Register::kSoftLimitOpen, false).ToString(),
            "$20=0");  // Section 2.3
  EXPECT_EQ(Assignment(Register::kHardLimitOpen, false).ToString(),
            "$21=0");  // Section 2.4

  // Section 2.5
  EXPECT_EQ(JointspaceMotionCommand({10, 15, 0, 0, 0, 10},
                                    CoordinateMode::kAbsolute, 2000).ToString(),
            "M21 G90 X10 Y15 C10 F2000");

  // Section 2.6
  EXPECT_EQ(JointspaceMotionCommand({10, -15, 0, 0, 0, -10},
                                    CoordinateMode::kRelative, 2000).ToString(),
            "M21 G91 X10 Y-15 C-10 F2000");

  // Section 2.7
  EXPECT_EQ(CartesianMotionCommand(
      {150, -30, 55, 0, 0, 0},
      CoordinateMode::kAbsolute, CartesianMotionMode::kFast, 2000).ToString(),
            "M20 G90 G0 X150 Y-30 Z55 A0 B0 C0 F2000");

  // Section 2.8
  EXPECT_EQ(CartesianMotionCommand(
      {150, -30, 55, 0, 0, 0}, CoordinateMode::kAbsolute,
      CartesianMotionMode::kInterpolated, 2000).ToString(),
            "M20 G90 G1 X150 Y-30 Z55 A0 B0 C0 F2000");

  // Section 2.9
  EXPECT_EQ(CartesianMotionCommand(
      {10, -30, 0, 0, 0, 0}, CoordinateMode::kRelative,
      CartesianMotionMode::kFast, 2000).ToString(),
            "M20 G91 G0 X10 Y-30 A0 B0 C0 F2000");

  // Section 2.10 *** NOTE:  The manual says G0 but I believe it is incorrect.
  EXPECT_EQ(CartesianMotionCommand(
      {10, -30, 0, 0, 0, 0}, CoordinateMode::kRelative,
      CartesianMotionMode::kInterpolated, 2000).ToString(),
            "M20 G91 G1 X10 Y-30 A0 B0 C0 F2000");

  EXPECT_EQ(RapidHome().ToString(), "$H");            // Section 2.11
  EXPECT_EQ(SequentialHome().ToString(), "$HH");      // Section 2.12

  EXPECT_EQ(SetPumpOpen().ToString(), "M3S1000");     // Section 2.13
  EXPECT_EQ(SetPumpClosed().ToString(), "M3S0");      // Section 2.13
  EXPECT_EQ(SetGripperOpen().ToString(), "M4E65");    // Section 2.14
  EXPECT_EQ(SetGripperClosed().ToString(), "M4E40");  // Section 2.14
}

}  // namespace driver
}  // namespace drake_mirobot
