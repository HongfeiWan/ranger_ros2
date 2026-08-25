// Copyright 2026 Hongfei Wan
// SPDX-License-Identifier: BSD-3-Clause

#include "ranger_base/ranger_state_semantics.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <ranger_msgs/msg/actuator_state.hpp>
#include <ranger_msgs/msg/driver_state.hpp>
#include <ranger_msgs/msg/motor_state.hpp>

namespace westonrobot {
namespace {

constexpr float kRangerMiniV3VoltageScale = 0.1F;

float Unknown() { return std::numeric_limits<float>::quiet_NaN(); }

}  // namespace

ranger_msgs::msg::ActuatorStateArray BuildActuatorStateMessage(
    const RangerActuatorState &state, const rclcpp::Time &stamp,
    bool ranger_mini_v3) {
  const std::array<float, 4> speeds = {
      state.motor_speeds.speed_1, state.motor_speeds.speed_2,
      state.motor_speeds.speed_3, state.motor_speeds.speed_4};
  const std::array<float, 4> angles = {
      state.motor_angles.angle_5, state.motor_angles.angle_6,
      state.motor_angles.angle_7, state.motor_angles.angle_8};

  ranger_msgs::msg::ActuatorStateArray message;
  message.header.stamp = stamp;
  message.states.reserve(8);
  for (std::size_t index = 0; index < 8; ++index) {
    ranger_msgs::msg::ActuatorState item;
    item.id = static_cast<uint32_t>(index + 1);
    item.driver.driver_voltage = state.actuator_ls_state[index].driver_voltage;
    item.driver.driver_temperature = state.actuator_ls_state[index].driver_temp;
    item.driver.motor_temperature = ranger_mini_v3
                                        ? Unknown()
                                        : state.actuator_ls_state[index].motor_temp;
    item.driver.driver_state = state.actuator_ls_state[index].driver_state;
    item.motor.current = state.actuator_hs_state[index].current;
    item.motor.pulse_count = state.actuator_hs_state[index].pulse_count;
    item.motor.rpm = state.actuator_hs_state[index].rpm;
    item.motor.motor_speeds = index < speeds.size() ? speeds[index] : Unknown();
    item.motor.motor_angles =
        index >= speeds.size() ? angles[index - speeds.size()] : Unknown();
    message.states.push_back(item);
  }
  return message;
}

sensor_msgs::msg::BatteryState BuildBatteryStateMessage(
    const RangerCommonSensorState &state, const rclcpp::Time &stamp,
    bool ranger_mini_v3, std::chrono::milliseconds freshness_timeout,
    SdkTimePoint now) {
  sensor_msgs::msg::BatteryState message;
  message.header.stamp = stamp;
  const bool received = state.time_stamp != SdkTimePoint{} &&
                        now >= state.time_stamp &&
                        now - state.time_stamp <= freshness_timeout;
  const float voltage = ranger_mini_v3
                            ? state.bms_basic_state.voltage *
                                  kRangerMiniV3VoltageScale
                            : state.bms_basic_state.voltage;
  const bool valid_voltage = received && std::isfinite(voltage) && voltage > 0.0F;
  message.voltage = valid_voltage ? voltage : Unknown();
  message.temperature = received ? state.bms_basic_state.temperature : Unknown();
  // Ranger Mini 3.0 documentation does not define the charge/discharge sign.
  message.current = ranger_mini_v3 ? Unknown() : state.bms_basic_state.current;
  message.charge = Unknown();
  message.capacity = Unknown();
  message.design_capacity = Unknown();
  const auto soc = state.bms_basic_state.battery_soc;
  message.percentage = received && soc <= 100 ? static_cast<float>(soc) / 100.0F
                                              : Unknown();
  message.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  message.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  message.power_supply_technology =
      ranger_mini_v3
          ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE
          : sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  message.present = valid_voltage;
  return message;
}

}  // namespace westonrobot
