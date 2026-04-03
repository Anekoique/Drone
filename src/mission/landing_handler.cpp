// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file landing_handler.cpp
/// @brief State machine for the landing phase: RTL, visual approach to H-marker
///        using camera tracking, controlled descent, and final LAND mode switch.

#include "drone/mission/landing_handler.hpp"

#include "drone/mission/frame_transforms.hpp"

#include <cmath>

namespace drone::mission
{

LandingHandler::LandingHandler(Subsystems & subs, const MissionConfig & config)
: subs_(subs), config_(config)
{
}

/// Execute the landing state machine.
/// States: rtl -> wait (18s for auto-descent) -> visual_approach (track H-marker
/// with surround search fallback) -> descent (slow velocity command) -> land.
FlyState LandingHandler::execute(float heading_rad, float start_yaw)
{
  switch (doland_state_) {
    case DolandState::rtl: {
      subs_.motors.switch_mode("RTL");
      state_timer_ = Timer{};
      doland_state_ = DolandState::wait;
      return FlyState::landing;
    }

    case DolandState::wait: {
      // Wait 18 seconds for RTL auto-descent
      if (state_timer_.elapsed() > 18.0) {
        subs_.motors.switch_mode("GUIDED");
        doland_state_ = DolandState::visual_approach;
        approach_initialized_ = false;
        state_timer_ = Timer{};
        target_loss_timer_ = Timer{};
      }
      return FlyState::landing;
    }

    case DolandState::visual_approach: {
      if (!approach_initialized_) {
        // Apply landing-specific PID/limits
        // Fly to scout position
        auto scout =
          compass_to_world(config_.landing.scout_x, config_.landing.scout_y + 0.3f, heading_rad);
        auto scout_world = world_to_start(scout.x(), scout.y(), start_yaw);

        Eigen::Vector4f target(scout_world.x(), scout_world.y(), config_.landing.scout_halt, 0);
        subs_.pos_control.go_to_position(target, 0.2f);
        approach_initialized_ = true;
      }

      // Check H-marker detection via tracker
      bool h_detected = tracker_ && tracker_->has_target(TargetClass::kH);

      if (h_detected) {
        // Visual servo approach to H-marker
        bool converged = catch_target(
          subs_, *tracker_, TargetClass::kH, config_.landing.tar_pixel, config_.landing.tar_z, 0,
          config_.landing.accuracy, config_.landing.pid, config_.landing.limits);
        target_loss_timer_ = Timer{};
        (void)converged;
      } else if (target_loss_timer_.elapsed() > 2.0) {
        // Surround search pattern when H not detected for >2 seconds
        float offset = static_cast<float>(surround_land_) * config_.landing.surround_step;
        auto search =
          compass_to_world(config_.landing.scout_x + offset, config_.landing.scout_y, heading_rad);
        auto search_world = world_to_start(search.x(), search.y(), start_yaw);

        Eigen::Vector4f target(search_world.x(), search_world.y(), config_.landing.scout_halt, 0);
        subs_.pos_control.go_to_position(target, 0.2f);

        surround_land_++;
        if (surround_land_ > static_cast<int>(config_.landing.surround_range)) {
          surround_land_ = -static_cast<int>(config_.landing.surround_range);
        }
        target_loss_timer_ = Timer{};
      }

      // Altitude check or timeout
      if (
        subs_.inav.position().z() < config_.landing.tar_z + 0.1f || state_timer_.elapsed() > 19.0) {
        doland_state_ = DolandState::descent;
        state_timer_ = Timer{};
      }

      return FlyState::landing;
    }

    case DolandState::descent: {
      // Descend at -0.2 m/s for 1 second (legacy Doland::end)
      bool done = subs_.pos_control.commander().send_velocity_timed(
        Eigen::Vector4f(0, 0, config_.landing.descent_speed, 0), config_.landing.descent_duration);

      if (done) {
        doland_state_ = DolandState::land;
      }
      return FlyState::landing;
    }

    case DolandState::land: {
      subs_.motors.switch_mode("LAND");
      return FlyState::finished;
    }
  }

  return FlyState::landing;
}

/// Alternative landing mode: fly back to the takeoff origin at 2m altitude,
/// wait for arrival or system standby, then switch to LAND.
FlyState LandingHandler::land_to_start()
{
  switch (lts_state_) {
    case LandToStartState::init: {
      subs_.motors.switch_mode("GUIDED");
      // Fly to origin at 2m altitude
      Eigen::Vector4f target(0, 0, 2.0f, 0);
      subs_.pos_control.go_to_position(target, 0.5f);
      state_timer_ = Timer{};
      lts_state_ = LandToStartState::wait;
      return FlyState::land_to_start;
    }

    case LandToStartState::wait: {
      // Maintain position
      Eigen::Vector4f target(0, 0, 2.0f, 0);
      subs_.pos_control.go_to_position(target, 0.5f);

      // Wait 19 seconds or until system is standby
      if (state_timer_.elapsed() > 19.0 || subs_.motors.system_status() == 3) {
        lts_state_ = LandToStartState::land;
      }
      return FlyState::land_to_start;
    }

    case LandToStartState::land: {
      subs_.motors.switch_mode("LAND");
      return FlyState::finished;
    }
  }

  return FlyState::land_to_start;
}

void LandingHandler::reset()
{
  doland_state_ = DolandState::rtl;
  surround_land_ = -static_cast<int>(config_.landing.surround_range);
  approach_initialized_ = false;
  state_timer_ = Timer{};
  target_loss_timer_ = Timer{};
}

void LandingHandler::reset_land_to_start()
{
  lts_state_ = LandToStartState::init;
}

}  // namespace drone::mission
