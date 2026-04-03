// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/control/pos_control.hpp"
#include "drone/drivers/gimbal.hpp"
#include "drone/drivers/inertial_nav.hpp"
#include "drone/drivers/motors.hpp"
#include "drone/drivers/node_base.hpp"
#include "drone/drivers/servo.hpp"
#include "drone/mission/airdrop_handler.hpp"
#include "drone/mission/landing_handler.hpp"
#include "drone/mission/mission_types.hpp"
#include "drone/mission/recon_handler.hpp"
#include "drone/mission/takeoff_handler.hpp"
#include "drone/mission/target_tracker.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

#include <string>

namespace drone::mission
{

/// Top-level drone mission node.
/// Owns all subsystems and handlers, runs state dispatch at 20 Hz.
class DroneNode : public NodeBase
{
public:
  /// @param mavros_ns MAVROS namespace (e.g. "/mavros/").
  /// @param config_dir Directory name containing mission YAML configs.
  DroneNode(const std::string & mavros_ns, const std::string & config_dir);

private:
  /// Main loop callback (20 Hz). Dispatches to current state handler.
  void timer_callback();

  // Subsystem ownership
  Motors motors_;
  InertialNav inav_;
  Servo servo_;
  Gimbal gimbal_;
  control::PosControl pos_control_;
  Subsystems subs_;

  // Mission
  MissionConfig config_;
  FlyState state_ = FlyState::init;
  FlyState prev_state_ = FlyState::init;

  // Target tracking
  TargetTracker tracker_;

  // Handlers
  TakeoffHandler takeoff_;
  AirdropHandler airdrop_;
  ReconHandler recon_;
  LandingHandler landing_;

  // Computed at init
  Eigen::Vector2f shot_origin_world_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f recon_origin_world_ = Eigen::Vector2f::Zero();

  // ROS 2
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  Timer shutdown_timer_;
};

}  // namespace drone::mission
