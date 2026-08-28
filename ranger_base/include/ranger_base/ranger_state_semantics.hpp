// Copyright 2026 Hongfei Wan
// SPDX-License-Identifier: BSD-3-Clause

#ifndef RANGER_STATE_SEMANTICS_HPP
#define RANGER_STATE_SEMANTICS_HPP

#include <chrono>
#include <cstdint>

#include <ranger_msgs/msg/actuator_state_array.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <ugv_sdk/details/interface/ranger_interface.hpp>

namespace westonrobot {

uint32_t BuildRangerMiniV3ErrorCode(const SystemStateMessage &state);

ranger_msgs::msg::ActuatorStateArray BuildActuatorStateMessage(
    const RangerActuatorState &state, const rclcpp::Time &stamp,
    bool ranger_mini_v3);

sensor_msgs::msg::BatteryState BuildBatteryStateMessage(
    const RangerCommonSensorState &state, const rclcpp::Time &stamp,
    bool ranger_mini_v3, std::chrono::milliseconds freshness_timeout,
    SdkTimePoint now = SdkClock::now());

}  // namespace westonrobot

#endif  // RANGER_STATE_SEMANTICS_HPP
