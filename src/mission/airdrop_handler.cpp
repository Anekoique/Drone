// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file airdrop_handler.cpp
/// @brief State machine for the airdrop mission phase: navigate to drop zone,
///        visually acquire circle targets, servo to center them in the camera
///        frame, and release payloads via servo commands.

#include "drone/mission/airdrop_handler.hpp"

#include "drone/mission/frame_transforms.hpp"

#include <cmath>

namespace drone::mission
{

AirdropHandler::AirdropHandler(Subsystems & subs, const MissionConfig & config)
: subs_(subs), config_(config)
{
}

/// Fly to the airdrop zone origin at shot altitude.
/// Uses a 12-second timer for arrival (legacy time-based transition).
FlyState AirdropHandler::goto_zone(const Eigen::Vector2f & zone_origin, float /*start_yaw*/)
{
  if (!goto_initialized_) {
    goto_timer_ = Timer{};
    goto_initialized_ = true;
  }

  // Fly to zone origin at shot altitude
  // Legacy: send_start_setpoint_command with shot_halt
  Eigen::Vector4f target(zone_origin.x(), zone_origin.y(), config_.shot_zone.altitude, 0);
  subs_.pos_control.go_to_position(target, 0.2f);

  // Time-based arrival (legacy: 12 seconds)
  if (goto_timer_.elapsed() > 12.0) {
    goto_initialized_ = false;
    return FlyState::airdrop;
  }

  return FlyState::goto_shot;
}

/// Run the airdrop sub-state machine.
/// Phases: init -> world_approach (fly to clustered target) -> pixel_approach
/// (visual servo centering) -> wait (servo settle) -> end (open all servos).
/// Handles two consecutive drops (shot_counter 1 and 2).
FlyState AirdropHandler::execute(
  const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw, bool fast_mode)
{
  switch (sub_state_) {
    case SubState::init: {
      shot_counter_ = 1;
      circle_counter_ = 0;
      wp_state_.reset();
      state_timer_ = Timer{};

      // Apply airdrop-specific PID/limits
      // (In Phase 7: actual PID override from config_.airdrop.pid)

      sub_state_ = SubState::world_approach;
      return FlyState::airdrop;
    }

    case SubState::world_approach: {
      // Fast mode: immediately fire servo without visual approach
      if (fast_mode) {
        subs_.servo.set_servo(10 + shot_counter_, config_.servo_open_pwm);
        state_timer_ = Timer{};
        sub_state_ = SubState::wait;
        return FlyState::airdrop;
      }

      // Use tracker if available for target-guided flight
      bool has_targets = tracker_ && !tracker_->get_clustered_targets().empty();
      bool has_circle = tracker_ && tracker_->has_target(TargetClass::kCircle);

      if (has_targets) {
        auto targets = tracker_->get_clustered_targets();
        int idx =
          std::min(static_cast<int>(shot_counter_ - 1), static_cast<int>(targets.size()) - 1);
        if (idx >= 0) {
          // Fly to clustered target at low altitude
          Eigen::Vector4f wp(
            static_cast<float>(targets[idx].position.x()),
            static_cast<float>(targets[idx].position.y()), config_.shot_zone.altitude_low, 0);
          subs_.pos_control.go_to_position(wp, config_.airdrop.radius);

          // Stable and close enough: transition to pixel approach
          Eigen::Vector3f pos = subs_.inav.position();
          float dist_xy =
            std::sqrt(std::pow(pos.x() - wp.x(), 2.0f) + std::pow(pos.y() - wp.y(), 2.0f));
          if (dist_xy < config_.airdrop.radius && state_timer_.elapsed() > 6.0) {
            sub_state_ = SubState::pixel_approach;
            state_timer_ = Timer{};
            return FlyState::airdrop;
          }
        }
        circle_counter_ = 0;
      } else {
        // No targets: increment circle miss counter
        if (!has_circle) {
          circle_counter_++;
        } else {
          circle_counter_ = 0;
        }
        // After 12 consecutive cycles without detection, patrol waypoints
        if (circle_counter_ >= 12) {
          waypoint_goto_next(
            subs_, config_.shot_zone, zone_origin, heading_rad, start_yaw, wp_state_, 5.0f);
        }
      }

      // Timeout: 70 seconds
      if (state_timer_.elapsed() > 70.0) {
        sub_state_ = SubState::end;
        state_timer_ = Timer{};
      }

      return FlyState::airdrop;
    }

    case SubState::pixel_approach: {
      // Visual servo: center target in camera frame using catch_target
      if (tracker_) {
        Eigen::Vector2f tar_pixel =
          (shot_counter_ == 1) ? config_.airdrop.tar_pixel_left : config_.airdrop.tar_pixel_right;
        bool converged = catch_target(
          subs_, *tracker_, TargetClass::kCircle, tar_pixel, config_.airdrop.tar_z, 0,
          config_.airdrop.accuracy, config_.airdrop.pid, config_.airdrop.limits);

        if (converged || state_timer_.elapsed() > 10.0) {
          subs_.servo.set_servo(10 + shot_counter_, config_.servo_open_pwm);
          state_timer_ = Timer{};
          sub_state_ = SubState::wait;
        }
      } else {
        // No tracker: skip pixel approach, fire directly
        subs_.servo.set_servo(10 + shot_counter_, config_.servo_open_pwm);
        state_timer_ = Timer{};
        sub_state_ = SubState::wait;
      }
      return FlyState::airdrop;
    }

    case SubState::wait: {
      // Wait 2 seconds for servo settle
      if (state_timer_.elapsed() > 2.0) {
        if (shot_counter_ <= 1) {
          shot_counter_++;
          circle_counter_ = 0;
          wp_state_.reset();
          state_timer_ = Timer{};
          sub_state_ = SubState::world_approach;
        } else {
          sub_state_ = SubState::end;
        }
      }
      return FlyState::airdrop;
    }

    case SubState::end: {
      // Safety: open all servos
      subs_.servo.set_servo(11, config_.servo_open_pwm);
      subs_.servo.set_servo(12, config_.servo_open_pwm);

      // Wait 2 seconds
      if (state_timer_.elapsed() > 2.0) {
        // Restore default limits
        subs_.pos_control.reset_limits();
        return FlyState::goto_recon;
      }

      return FlyState::airdrop;
    }
  }

  return FlyState::airdrop;
}

void AirdropHandler::reset()
{
  sub_state_ = SubState::init;
  shot_counter_ = 1;
  circle_counter_ = 0;
  wp_state_.reset();
  goto_initialized_ = false;
}

}  // namespace drone::mission
