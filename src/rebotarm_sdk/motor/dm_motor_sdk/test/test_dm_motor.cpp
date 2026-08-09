#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <variant>

#include "rebotarm_dm_motor_sdk/dm_motor.hpp"
#include "gtest/gtest.h"

namespace {

using rebotarm_dm_motor_sdk::CanFrame;
using rebotarm_dm_motor_sdk::minimum_jerk;
using rebotarm_dm_motor_sdk::MitCommand;
using rebotarm_dm_motor_sdk::MotorConfig;
using rebotarm_dm_motor_sdk::MotorModel;
using rebotarm_dm_motor_sdk::ParameterValue;
using rebotarm_dm_motor_sdk::PositionForceCommand;
using rebotarm_dm_motor_sdk::PositionVelocityCommand;
using rebotarm_dm_motor_sdk::Protocol;
using rebotarm_dm_motor_sdk::Register;
using rebotarm_dm_motor_sdk::StatusCommand;

const MotorConfig kMotor{"joint1", 0x01, 0x11, MotorModel::kDm4310};

TEST(DmProtocolTest, EncodesStatusCommands) {
  for (const auto command :
       {StatusCommand::kClearError, StatusCommand::kEnable,
        StatusCommand::kDisable, StatusCommand::kSetZero}) {
    const auto frame = Protocol::status(kMotor, command);
    EXPECT_EQ(frame.id, 0x01);
    EXPECT_EQ(frame.size, 8);
    for (std::size_t i = 0; i < 7; ++i) {
      EXPECT_EQ(frame.data[i], 0xFF);
    }
    EXPECT_EQ(frame.data[7], static_cast<std::uint8_t>(command));
  }
}

TEST(DmProtocolTest, EncodesMitBitFields) {
  const auto frame = Protocol::mit(kMotor, MitCommand{});
  EXPECT_EQ(frame.id, 0x01);
  EXPECT_EQ(frame.size, 8);
  EXPECT_EQ(frame.data[0], 0x7F);
  EXPECT_EQ(frame.data[1], 0xFF);
  EXPECT_EQ(frame.data[2], 0x7F);
  EXPECT_EQ(frame.data[3] >> 4, 0x0F);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6] >> 4, 0x00);
  EXPECT_EQ(frame.data[7], 0xFF);
}

TEST(DmProtocolTest, EncodesFloatControlModesLittleEndian) {
  const auto posvel =
      Protocol::position_velocity(kMotor, PositionVelocityCommand{1.0F, 2.0F});
  EXPECT_EQ(posvel.id, 0x101);
  EXPECT_EQ(posvel.size, 8);
  EXPECT_EQ(posvel.data, (std::array<std::uint8_t, 8>{0x00, 0x00, 0x80, 0x3F,
                                                      0x00, 0x00, 0x00, 0x40}));

  const auto velocity = Protocol::velocity(kMotor, 1.0F);
  EXPECT_EQ(velocity.id, 0x201);
  EXPECT_EQ(velocity.size, 4);
  EXPECT_EQ(velocity.data[0], 0x00);
  EXPECT_EQ(velocity.data[1], 0x00);
  EXPECT_EQ(velocity.data[2], 0x80);
  EXPECT_EQ(velocity.data[3], 0x3F);
}

TEST(DmProtocolTest, EncodesPositionForceScaling) {
  const auto frame =
      Protocol::position_force(kMotor, PositionForceCommand{1.0F, 5.0, 0.2});
  EXPECT_EQ(frame.id, 0x301);
  EXPECT_EQ(frame.size, 8);
  EXPECT_EQ(frame.data[4], 0xF4);
  EXPECT_EQ(frame.data[5], 0x01);
  EXPECT_EQ(frame.data[6], 0xD0);
  EXPECT_EQ(frame.data[7], 0x07);
}

TEST(DmProtocolTest, EncodesManagementFramesWithDocumentedDlc) {
  const auto refresh = Protocol::refresh(kMotor);
  EXPECT_EQ(refresh.id, 0x7FF);
  EXPECT_EQ(refresh.size, 8);
  EXPECT_EQ(refresh.data,
            (std::array<std::uint8_t, 8>{0x01, 0x00, 0xCC, 0, 0, 0, 0, 0}));

  const auto read = Protocol::read_parameter(kMotor, Register::kControlMode);
  EXPECT_EQ(read.size, 4);
  EXPECT_EQ(read.data[0], 0x01);
  EXPECT_EQ(read.data[1], 0x00);
  EXPECT_EQ(read.data[2], 0x33);
  EXPECT_EQ(read.data[3], 0x0A);

  const auto write = Protocol::write_parameter(
      kMotor, Register::kControlMode, ParameterValue{std::uint32_t{2}});
  EXPECT_EQ(write.size, 8);
  EXPECT_EQ(write.data, (std::array<std::uint8_t, 8>{0x01, 0x00, 0x55, 0x0A,
                                                     0x02, 0, 0, 0}));

  const auto store = Protocol::store_parameters(kMotor);
  EXPECT_EQ(store.size, 4);
  EXPECT_EQ(store.data[2], 0xAA);
  EXPECT_EQ(store.data[3], 0x01);
}

TEST(DmProtocolTest, RejectsWrongParameterTypesAndReadOnlyWrites) {
  EXPECT_THROW(Protocol::write_parameter(kMotor, Register::kControlMode,
                                         ParameterValue{1.0F}),
               std::invalid_argument);
  EXPECT_THROW(Protocol::write_parameter(kMotor, Register::kPositionMaximum,
                                         ParameterValue{std::uint32_t{1}}),
               std::invalid_argument);
  EXPECT_THROW(Protocol::write_parameter(kMotor, Register::kBusVoltage,
                                         ParameterValue{24.0F}),
               std::invalid_argument);
}

TEST(DmProtocolTest, DecodesObservedStateFeedback) {
  CanFrame frame;
  frame.id = 0x11;
  frame.size = 8;
  frame.data = {0x11, 0x7F, 0xFC, 0x7F, 0xF7, 0xFB, 0x1B, 0x1A};
  const auto result = Protocol::decode_state(kMotor, frame);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->motor_id, 1);
  EXPECT_EQ(result->status, 1);
  EXPECT_NEAR(result->position, 0.0, 0.01);
  EXPECT_NEAR(result->velocity, 0.0, 0.02);
  // The captured neutral sample is four 12-bit counts below the midpoint.
  EXPECT_NEAR(result->effort, 0.0, 0.03);
  EXPECT_EQ(result->mos_temperature, 27);
  EXPECT_EQ(result->rotor_temperature, 26);
}

TEST(DmProtocolTest, DecodesIntegerAndFloatParameterResponses) {
  CanFrame integer_frame;
  integer_frame.id = 0x11;
  integer_frame.size = 8;
  integer_frame.data = {0x01, 0x00, 0x33, 0x0A, 0x02, 0, 0, 0};
  const auto integer_result = Protocol::decode_parameter(kMotor, integer_frame);
  ASSERT_TRUE(integer_result.has_value());
  ASSERT_TRUE(std::holds_alternative<std::uint32_t>(integer_result->value));
  EXPECT_EQ(std::get<std::uint32_t>(integer_result->value), 2U);

  CanFrame float_frame;
  float_frame.id = 0x11;
  float_frame.size = 8;
  float_frame.data = {0x01, 0x00, 0x33, 0x15, 0x00, 0x00, 0x48, 0x41};
  const auto float_result = Protocol::decode_parameter(kMotor, float_frame);
  ASSERT_TRUE(float_result.has_value());
  ASSERT_TRUE(std::holds_alternative<float>(float_result->value));
  EXPECT_FLOAT_EQ(std::get<float>(float_result->value), 12.5F);
}

TEST(DmProtocolTest, DecodesStoreAcknowledgement) {
  CanFrame frame;
  frame.id = 0x11;
  frame.size = 4;
  frame.data = {0x01, 0x00, 0xAA, 0x01, 0, 0, 0, 0};
  EXPECT_TRUE(Protocol::decode_store_ack(kMotor, frame));
  frame.data[0] = 0x02;
  EXPECT_FALSE(Protocol::decode_store_ack(kMotor, frame));
}

TEST(DmProtocolTest, RejectsNonFiniteCommands) {
  auto command = MitCommand{};
  command.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Protocol::mit(kMotor, command), std::invalid_argument);
}

TEST(DmProtocolTest, UsesManualAndSdkModelLimits) {
  const auto dm4310 = Protocol::limits(MotorModel::kDm4310);
  EXPECT_DOUBLE_EQ(dm4310.position, 12.5);
  EXPECT_DOUBLE_EQ(dm4310.velocity, 30.0);
  EXPECT_DOUBLE_EQ(dm4310.effort, 10.0);

  const auto dm4340 = Protocol::limits(MotorModel::kDm4340);
  EXPECT_DOUBLE_EQ(dm4340.velocity, 8.0);
  EXPECT_DOUBLE_EQ(dm4340.effort, 28.0);

  const auto dm4340_48v = Protocol::limits(MotorModel::kDm4340_48V);
  EXPECT_DOUBLE_EQ(dm4340_48v.velocity, 10.0);
  EXPECT_DOUBLE_EQ(dm4340_48v.effort, 28.0);
}

TEST(DmProtocolTest, GeneratesMinimumJerkBoundaryConditions) {
  const auto duration = std::chrono::duration<double>(8.0);
  const auto start =
      minimum_jerk(0.2, 0.0, std::chrono::duration<double>(0.0), duration);
  EXPECT_DOUBLE_EQ(start.position, 0.2);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);

  const auto middle = minimum_jerk(0.2, 0.0, duration / 2.0, duration);
  EXPECT_NEAR(middle.position, 0.1, 1e-12);
  EXPECT_LT(middle.velocity, 0.0);

  const auto end = minimum_jerk(0.2, 0.0, duration, duration);
  EXPECT_NEAR(end.position, 0.0, 1e-12);
  EXPECT_NEAR(end.velocity, 0.0, 1e-12);
  EXPECT_NEAR(end.acceleration, 0.0, 1e-12);
}

} // namespace
