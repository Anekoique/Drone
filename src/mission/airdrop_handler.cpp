#include "drone/mission/airdrop_handler.hpp"

#include "drone/mission/frame_transforms.hpp"

#include <cmath>

namespace drone::mission
{

AirdropHandler::AirdropHandler(Subsystems & subs, const MissionConfig & config)
: subs_(subs), config_(config)
{
}

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
      // TODO Phase 7: get clustered targets from detector
      // For now, fly waypoint patrol pattern at altitude_low

      // Fast mode: immediately fire servo
      if (fast_mode) {
        subs_.servo.set_servo(10 + shot_counter_, config_.servo_open_pwm);
        state_timer_ = Timer{};
        sub_state_ = SubState::wait;
        return FlyState::airdrop;
      }

      // Circle detection fallback: after 12 cycles with no detection, patrol
      // TODO Phase 7: actual detector integration
      // For now, always patrol since we have no detector
      bool all_done = waypoint_goto_next(
        subs_, config_.shot_zone, zone_origin, heading_rad, start_yaw, wp_state_, 5.0f);

      if (all_done) {
        sub_state_ = SubState::end;
        state_timer_ = Timer{};
      }

      // Timeout: 70 seconds
      if (state_timer_.elapsed() > 70.0) {
        sub_state_ = SubState::end;
        state_timer_ = Timer{};
      }

      return FlyState::airdrop;
    }

    case SubState::pixel_approach: {
      // TODO Phase 7: catch_target() visual servo approach
      // Stub: transition to wait without firing servo (Phase 7 adds visual servo logic)
      state_timer_ = Timer{};
      sub_state_ = SubState::wait;
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
