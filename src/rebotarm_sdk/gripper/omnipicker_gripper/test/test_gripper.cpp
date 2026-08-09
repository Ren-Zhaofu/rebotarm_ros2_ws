#include "rebotarm_gripper_sdk/gripper.hpp"

#include <gtest/gtest.h>

using rebotarm_gripper_sdk::CanFrame;
using rebotarm_gripper_sdk::Command;
using rebotarm_gripper_sdk::Protocol;

TEST(GripperProtocol, UsesHardwareDefaultCanId) {
  const auto frame = Protocol::command(Command{0.5, 1.0, 0.25, 0.0, 1.0});
  EXPECT_EQ(frame.id, 0x07);
  EXPECT_EQ(frame.size, 8);
  EXPECT_EQ(frame.data,
            (std::array<std::uint8_t, 8>{0x00, 0x80, 0xFF, 0x40,
                                         0x00, 0xFF, 0x00, 0x00}));
}

TEST(GripperProtocol, RejectsUnsafeNormalizedValues) {
  EXPECT_THROW(Protocol::command(Command{-0.01, 1.0, 1.0, 1.0, 1.0}),
               std::invalid_argument);
  EXPECT_THROW(Protocol::command(Command{0.0, 1.01, 1.0, 1.0, 1.0}),
               std::invalid_argument);
}

TEST(GripperProtocol, BuildsStopFrame) {
  const auto frame = Protocol::stop();
  EXPECT_EQ(frame.id, 0x07);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
}

TEST(GripperProtocol, DecodesMatchingEightByteFeedback) {
  CanFrame frame;
  frame.id = 0x07;
  frame.size = 8;
  frame.data = {0x02, 0x03, 0xFF, 0x80, 0x40, 0x00, 0x00, 0x00};
  const auto state = Protocol::decode_state(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->fault_code, 0x02);
  EXPECT_EQ(state->status, 0x03);
  EXPECT_DOUBLE_EQ(state->position, 1.0);
  EXPECT_NEAR(state->velocity, 128.0 / 255.0, 1e-12);
  EXPECT_NEAR(state->force, 64.0 / 255.0, 1e-12);
}

TEST(GripperProtocol, IgnoresOtherIdsAndMalformedFrames) {
  CanFrame frame;
  frame.id = 0x08;
  frame.size = 8;
  EXPECT_FALSE(Protocol::decode_state(frame).has_value());
  frame.id = 0x07;
  frame.size = 5;
  EXPECT_FALSE(Protocol::decode_state(frame).has_value());
}
