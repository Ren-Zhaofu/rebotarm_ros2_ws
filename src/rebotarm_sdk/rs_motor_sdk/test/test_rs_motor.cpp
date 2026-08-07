#include <cmath>

#include "rs_motor_sdk/rs_motor.hpp"
#include "gtest/gtest.h"

namespace {

using rs_motor_sdk::CommunicationType;
using rs_motor_sdk::MotorConfig;
using rs_motor_sdk::MotorModel;
using rs_motor_sdk::Protocol;

const MotorConfig kMotor{"joint1", 0x01, 0xFD, MotorModel::kRs06};

TEST(RsProtocolTest, BuildsExtendedStatusCommands) {
  const auto enable = Protocol::enable(kMotor);
  EXPECT_TRUE(enable.extended);
  EXPECT_EQ(enable.id, 0x0300FD01U);
  EXPECT_EQ(enable.size, 8);

  const auto disable = Protocol::disable(kMotor, true);
  EXPECT_EQ(disable.id, 0x0400FD01U);
  EXPECT_EQ(disable.data[0], 1U);
}

TEST(RsProtocolTest, BuildsActiveReportCommand) {
  const auto frame = Protocol::active_report(kMotor, true);
  EXPECT_EQ(frame.id, 0x1800FD01U);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{1, 2, 3, 4, 5, 6, 1, 0}));
}

TEST(RsProtocolTest, EncodesNeutralMitCommand) {
  const auto frame = Protocol::mit(kMotor, {});
  EXPECT_EQ(Protocol::communication_type(frame),
            CommunicationType::kMitControl);
  EXPECT_EQ(frame.id, 0x017FFF01U);
  EXPECT_EQ(frame.data[0], 0x7F);
  EXPECT_EQ(frame.data[1], 0xFF);
  EXPECT_EQ(frame.data[2], 0x7F);
  EXPECT_EQ(frame.data[3], 0xFF);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6], 0x00);
  EXPECT_EQ(frame.data[7], 0x00);
}

TEST(RsProtocolTest, EncodesFloatParameterWriteLittleEndian) {
  const auto frame = Protocol::write_parameter_f32(kMotor, 0x7016, 1.0F);
  EXPECT_EQ(frame.id, 0x1200FD01U);
  EXPECT_EQ(frame.data[0], 0x16);
  EXPECT_EQ(frame.data[1], 0x70);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6], 0x80);
  EXPECT_EQ(frame.data[7], 0x3F);
}

TEST(RsProtocolTest, DecodesParameterResponse) {
  rs_motor_sdk::CanFrame frame;
  frame.id = 0x110001FDU;
  frame.size = 8;
  frame.data = {0x05, 0x70, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  const auto response = Protocol::decode_parameter(kMotor, frame);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->motor_id, 1U);
  EXPECT_EQ(response->index, 0x7005U);
  EXPECT_EQ(response->value,
            (std::array<std::uint8_t, 4>{0x02, 0x00, 0x00, 0x00}));
}

TEST(RsProtocolTest, DecodesFeedback) {
  rs_motor_sdk::CanFrame frame;
  frame.id = 0x020001FD;
  frame.size = 8;
  frame.data = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C};
  const auto state = Protocol::decode_feedback(kMotor, frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->motor_id, 1U);
  EXPECT_EQ(state->fault, 0U);
  EXPECT_EQ(state->mode, 0U);
  EXPECT_NEAR(state->position, 0.0, 0.001);
  EXPECT_NEAR(state->velocity, 0.0, 0.001);
  EXPECT_NEAR(state->effort, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(state->temperature, 30.0);
  EXPECT_EQ(state->source, CommunicationType::kFeedback);
  EXPECT_EQ(state->raw_can_id, frame.id);
}

TEST(RsProtocolTest, DecodesPeriodicActiveReport) {
  rs_motor_sdk::CanFrame frame;
  frame.id = 0x180001FD;
  frame.size = 8;
  frame.data = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C};
  const auto state = Protocol::decode_feedback(kMotor, frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->fault, 0U);
  EXPECT_EQ(state->source, CommunicationType::kActiveReport);
  EXPECT_EQ(state->raw_can_id, frame.id);
}

TEST(RsTrajectoryTest, MinimumJerkHasSmoothEndpointsAndMidpoint) {
  const auto start = rs_motor_sdk::minimum_jerk(1.0, 0.0, 10.0, 0.0);
  EXPECT_DOUBLE_EQ(start.position, 1.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);

  const auto midpoint = rs_motor_sdk::minimum_jerk(1.0, 0.0, 10.0, 5.0);
  EXPECT_DOUBLE_EQ(midpoint.position, 0.5);
  EXPECT_NEAR(midpoint.velocity, -0.1875, 1e-12);

  const auto end = rs_motor_sdk::minimum_jerk(1.0, 0.0, 10.0, 10.0);
  EXPECT_DOUBLE_EQ(end.position, 0.0);
  EXPECT_DOUBLE_EQ(end.velocity, 0.0);
}

TEST(RsTrajectoryTest, MinimumJerkClampsTimeAndRejectsInvalidDuration) {
  EXPECT_DOUBLE_EQ(rs_motor_sdk::minimum_jerk(1.0, 0.0, 10.0, -1.0).position,
                   1.0);
  EXPECT_DOUBLE_EQ(rs_motor_sdk::minimum_jerk(1.0, 0.0, 10.0, 11.0).position,
                   0.0);
  EXPECT_THROW(rs_motor_sdk::minimum_jerk(0.0, 1.0, 0.0, 0.0),
               std::invalid_argument);
}

} // namespace
