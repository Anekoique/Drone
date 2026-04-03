// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/math/utils.hpp"

#include <Eigen/Core>

#include <cstdint>

namespace drone
{

/// S-curve trajectory planner for smooth jerk-limited motion along a 3D track.
/// Computes a time-optimal 7-segment profile respecting snap, jerk, accel, and velocity limits.
class SCurve
{
public:
  SCurve();

  /// Reset trajectory to initial state.
  void init();

  /// Solve the 1D S-curve path parameters for given kinematic constraints.
  /// @param Sm Snap maximum.
  /// @param Jm Jerk maximum.
  /// @param V0 Initial velocity.
  /// @param Am Acceleration maximum.
  /// @param Vm Velocity maximum.
  /// @param L Total path length.
  /// @param Jm_out Computed jerk limit.
  /// @param tj_out Jerk ramp time.
  /// @param t2_out Constant-acceleration time.
  /// @param t4_out Cruise time.
  /// @param t6_out Deceleration constant-acceleration time.
  static void calculate_path(
    float Sm, float Jm, float V0, float Am, float Vm, float L, float & Jm_out, float & tj_out,
    float & t2_out, float & t4_out, float & t6_out);

  /// Plan a 3D trajectory between two waypoints with kinematic limits.
  /// @param origin Start position in world frame.
  /// @param destination End position in world frame.
  void calculate_track(
    const Eigen::Vector3f & origin, const Eigen::Vector3f & destination, float speed_xy,
    float speed_up, float speed_down, float accel_xy, float accel_z, float snap_maximum,
    float jerk_maximum);

  /// Set maximum cruise speeds for XY, up, and down.
  void set_speed_max(float speed_xy, float speed_up, float speed_down);

  /// Set and return the origin speed limit.
  /// @return Clamped origin speed.
  float set_origin_speed_max(float speed);

  /// Set the destination speed limit.
  void set_destination_speed_max(float speed);

  /// Advance the target position along the track by dt seconds.
  /// @param prev_leg Previous trajectory leg (for blending).
  /// @param next_leg Next trajectory leg (for corner handling).
  /// @param wp_radius Waypoint acceptance radius.
  /// @param accel_corner Corner acceleration limit.
  /// @param fast_waypoint If true, use fast waypoint switching.
  /// @param dt Time step in seconds.
  /// @param target_pos Output position.
  /// @param target_vel Output velocity.
  /// @param target_accel Output acceleration.
  /// @return true if the leg is finished.
  bool advance_target_along_track(
    SCurve & prev_leg, SCurve & next_leg, float wp_radius, float accel_corner, bool fast_waypoint,
    float dt, Eigen::Vector3f & target_pos, Eigen::Vector3f & target_vel,
    Eigen::Vector3f & target_accel);

  /// @return true if the trajectory has completed.
  bool finished();

private:
  void move_from_pos_vel_accel(
    float dt, Eigen::Vector3f & pos, Eigen::Vector3f & vel, Eigen::Vector3f & accel);
  void move_to_pos_vel_accel(
    float dt, Eigen::Vector3f & pos, Eigen::Vector3f & vel, Eigen::Vector3f & accel);
  void move_from_time_pos_vel_accel(
    float t, Eigen::Vector3f & pos, Eigen::Vector3f & vel, Eigen::Vector3f & accel);

  float get_speed_along_track() { return vel_max; }
  float get_accel_along_track() { return accel_max; }
  const Eigen::Vector3f & get_track() { return track; }
  float get_time_elapsed() { return time; }
  float time_end();
  float get_time_remaining();
  float get_accel_finished_time();
  bool braking();
  float time_turn_in();
  float time_turn_out();
  void advance_time(float dt);

  void get_jerk_accel_vel_pos_at_time(
    float time_now, float & Jt_out, float & At_out, float & Vt_out, float & Pt_out) const;

  void calc_javp_for_segment_const_jerk(
    float time_now, float J0, float A0, float V0, float P0, float & Jt, float & At, float & Vt,
    float & Pt) const;

  void calc_javp_for_segment_incr_jerk(
    float time_now, float tj, float Jm, float A0, float V0, float P0, float & Jt, float & At,
    float & Vt, float & Pt) const;

  void calc_javp_for_segment_decr_jerk(
    float time_now, float tj, float Jm, float A0, float V0, float P0, float & Jt, float & At,
    float & Vt, float & Pt) const;

  void add_segments(float L);
  void add_segments_jerk(uint8_t & seg_pnt, float Jm, float tj, float Tcj);
  void add_segment_const_jerk(uint8_t & seg_pnt, float J0, float tin);
  void add_segment_incr_jerk(uint8_t & seg_pnt, float Jm, float tj);
  void add_segment_decr_jerk(uint8_t & seg_pnt, float Jm, float tj);

  void set_kinematic_limits(
    const Eigen::Vector3f & origin, const Eigen::Vector3f & destination, float speed_xy,
    float speed_up, float speed_down, float accel_xy, float accel_z);

  bool valid();

  enum class SegmentType
  {
    CONSTANT_JERK,
    POSITIVE_JERK,
    NEGATIVE_JERK
  };

  void add_segment(
    uint8_t & seg_pnt, float end_time, SegmentType seg_type, float jerk_ref, float end_accel,
    float end_vel, float end_pos);

  float snap_max = 0;
  float jerk_max = 0;
  float accel_max = 0;
  float vel_max = 0;
  float time = 0;
  float position_sq = 0;

  static constexpr uint8_t segments_max = 23;
  uint8_t num_segs = 0;

  struct Segment
  {
    float jerk_ref = 0;
    SegmentType seg_type = SegmentType::CONSTANT_JERK;
    float end_time = 0;
    float end_accel = 0;
    float end_vel = 0;
    float end_pos = 0;
  };
  Segment segment[segments_max] = {};

  Eigen::Vector3f track = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_unit = Eigen::Vector3f::Zero();
};

}  // namespace drone
