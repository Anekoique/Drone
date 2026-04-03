// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file drone_node.cpp
/// @brief Main ROS2 node that orchestrates the complete mission lifecycle.
///        Runs a 50ms timer loop dispatching to takeoff, airdrop, recon, and
///        landing handlers based on the current FlyState.

#include "drone/mission/drone_node.hpp"

#include "drone/mission/frame_transforms.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace drone::mission
{

DroneNode::DroneNode(const std::string & mavros_ns, const std::string & /*config_dir*/)
: NodeBase("drone_node", mavros_ns),
  motors_(*this, mavros_ns),
  inav_(*this, mavros_ns),
  servo_(*this, mavros_ns),
  gimbal_(*this, mavros_ns),
  pos_control_(*this, inav_, "pos_control.yaml"),
  subs_{motors_, inav_, pos_control_, servo_, gimbal_},
  config_(MissionConfig::load("mission.yaml", "airdrop.yaml", "landing.yaml")),
  tracker_(inav_, gimbal_, config_),
  takeoff_(subs_, config_),
  airdrop_(subs_, config_),
  recon_(subs_, config_),
  landing_(subs_, config_)
{
  // Wire tracker into handlers
  airdrop_.set_tracker(&tracker_);
  landing_.set_tracker(&tracker_);

  state_pub_ = create_publisher<std_msgs::msg::Int32>("drone/state", 10);
  loop_timer_ = create_wall_timer(50ms, std::bind(&DroneNode::timer_callback, this));
}

/// Main loop at 20 Hz. Guards against disconnected FCU or uninitialized position,
/// resets handlers on state transitions, updates the camera model, and dispatches
/// to the active mission handler based on FlyState.
void DroneNode::timer_callback()
{
  // System readiness guard
  if (!motors_.connected()) return;
  if (state_ != FlyState::init && subs_.inav.position().norm() < 1e-6f) return;

  // State entry reset (C-8)
  if (state_ != prev_state_) {
    switch (state_) {
      case FlyState::airdrop:
        airdrop_.reset();
        break;
      case FlyState::recon_patrol:
        recon_.reset();
        break;
      case FlyState::landing:
        landing_.reset();
        break;
      case FlyState::land_to_start:
        landing_.reset_land_to_start();
        break;
      default:
        break;
    }
    prev_state_ = state_;
  }

  // Update camera model with current pose and feed detections
  tracker_.update_camera_pose();
  // Note: detector inference and feed_detections() are called externally
  // or via a detection subscription callback (Phase 7 integration)

  float heading = config_.heading_compass_rad;
  float start_yaw = takeoff_.start_yaw();

  // State dispatch
  switch (state_) {
    case FlyState::init:
      // Wait for valid position fix before initializing
      if (subs_.inav.position().norm() < 1e-6f) break;
      state_ = takeoff_.initialize();
      // Compute zone origins after init captures start position
      if (state_ == FlyState::takeoff) {
        shot_origin_world_ =
          zone_origin_to_world(config_.shot_zone.dx, config_.shot_zone.dy, heading, start_yaw);
        recon_origin_world_ =
          zone_origin_to_world(config_.recon_zone.dx, config_.recon_zone.dy, heading, start_yaw);
      }
      break;

    case FlyState::takeoff:
      state_ = takeoff_.takeoff();
      break;

    case FlyState::goto_shot:
      state_ = airdrop_.goto_zone(shot_origin_world_, start_yaw);
      break;

    case FlyState::airdrop:
      state_ = airdrop_.execute(shot_origin_world_, heading, start_yaw, fast_mode());
      break;

    case FlyState::goto_recon:
      state_ = recon_.goto_zone(recon_origin_world_, start_yaw);
      break;

    case FlyState::recon_patrol:
      state_ = recon_.patrol(recon_origin_world_, heading, start_yaw);
      break;

    case FlyState::landing:
      state_ = landing_.execute(heading, start_yaw);
      break;

    case FlyState::land_to_start:
      state_ = landing_.land_to_start();
      break;

    case FlyState::finished:
      if (shutdown_timer_.elapsed() > 3.0) {
        rclcpp::shutdown();
      }
      break;
  }

  // Publish state
  auto msg = std_msgs::msg::Int32();
  msg.data = fly_state_to_int(state_);
  state_pub_->publish(msg);
}

}  // namespace drone::mission
