// Copyright 2026 Hongfei Wan
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>

#include "gtest/gtest.h"
#include "ranger_base/ranger_state_semantics.hpp"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

using namespace westonrobot;

TEST(RangerStateSemantics, PreservesEightDistinctActuatorSlots) {
  RangerActuatorState state{};
  state.motor_speeds = {1.0F, 2.0F, 3.0F, 4.0F};
  state.motor_angles = {5.0F, 6.0F, 7.0F, 8.0F};
  for (std::size_t index = 0; index < 8; ++index) {
    state.actuator_hs_state[index].rpm = static_cast<int16_t>(100 + index);
    state.actuator_ls_state[index].driver_voltage =
        static_cast<float>(40 + index);
  }

  const auto message = BuildActuatorStateMessage(state, {});
  ASSERT_EQ(message.states.size(), 8U);
  for (std::size_t index = 0; index < 8; ++index) {
    EXPECT_EQ(message.states[index].id, index + 1);
    EXPECT_EQ(message.states[index].motor.rpm, 100 + index);
    EXPECT_FLOAT_EQ(message.states[index].driver.driver_voltage, 40.0F + index);
  }
  for (std::size_t index = 0; index < 4; ++index) {
    EXPECT_FLOAT_EQ(message.states[index].motor.motor_speeds, 1.0F + index);
    EXPECT_TRUE(std::isnan(message.states[index].motor.motor_angles));
    EXPECT_FLOAT_EQ(message.states[index + 4].motor.motor_angles, 5.0F + index);
    EXPECT_TRUE(std::isnan(message.states[index + 4].motor.motor_speeds));
  }
}

TEST(RangerStateSemantics, PublishesStandardMiniV3BatterySemantics) {
  RangerCommonSensorState state{};
  state.time_stamp = SdkClock::now();
  state.bms_basic_state.voltage = 538.0F;
  state.bms_basic_state.current = 17.0F;
  state.bms_basic_state.temperature = 43.8F;
  state.bms_basic_state.battery_soc = 36;

  const auto message = BuildBatteryStateMessage(state, {}, true);
  EXPECT_FLOAT_EQ(message.voltage, 53.8F);
  EXPECT_TRUE(std::isnan(message.current));
  EXPECT_FLOAT_EQ(message.temperature, 43.8F);
  EXPECT_FLOAT_EQ(message.percentage, 0.36F);
  EXPECT_TRUE(message.present);
  EXPECT_EQ(message.power_supply_technology,
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE);
}

TEST(RangerStateSemantics, RejectsMissingOrInvalidBatteryFeedback) {
  RangerCommonSensorState missing{};
  auto message = BuildBatteryStateMessage(missing, {}, true);
  EXPECT_FALSE(message.present);
  EXPECT_TRUE(std::isnan(message.voltage));
  EXPECT_TRUE(std::isnan(message.percentage));

  missing.time_stamp = SdkClock::now();
  missing.bms_basic_state.voltage = 538.0F;
  missing.bms_basic_state.battery_soc = 101;
  message = BuildBatteryStateMessage(missing, {}, true);
  EXPECT_TRUE(message.present);
  EXPECT_TRUE(std::isnan(message.percentage));
}
