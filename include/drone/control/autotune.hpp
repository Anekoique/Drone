// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstdint>

namespace drone
{

/// Maximum number of concurrent autotune objects.
constexpr int kAutoTuneMaxObjects = 4;

/// Opaque handle to an autotune session.
using TuneId = int32_t;

/// Controller type for autotune output.
enum class ControllerType : int
{
  kPI,
  kPID,
};

/// Autotune state machine state.
enum class TuneState : int
{
  kInit = 0,
  kStartPoint,
  kRunning,
  kFail,
  kSuccess,
};

/// Relay feedback driver direction.
enum class DriverAction : int
{
  kPositive,
  kNegative,
};

/// Configuration for an autotune session.
struct TuneConfig
{
  ControllerType ctrl_type = ControllerType::kPID;
  DriverAction action_type = DriverAction::kPositive;
  float max_output_step = 50.0f;     ///< Maximum relay output step
  float min_output_step = 0.0f;      ///< Minimum relay output step
  uint32_t hysteresis_count = 5;     ///< Cycles before declaring convergence
  float setpoint = 50.0f;            ///< Target setpoint for relay feedback
  float amp_std_deviation = 0.0f;    ///< Acceptable amplitude std deviation
  float cycle_std_deviation = 0.0f;  ///< Acceptable cycle-time std deviation
};

/// Initialize the autotune subsystem. Call once at startup.
void tune_init();

/// Create a new autotune session.
/// @param config Pointer to configuration struct.
/// @return Session ID, or negative on failure.
TuneId tune_new(const TuneConfig * config);

/// Release an autotune session.
/// @param id Session ID to release.
/// @return true on success.
bool tune_release(TuneId id);

/// Run one autotune iteration.
/// @param id Session ID.
/// @param feedback Current process measurement.
/// @param output Output relay value (written by the function).
/// @param delay_ms Delay between iterations in milliseconds.
/// @return Current autotune state.
TuneState tune_work(TuneId id, float feedback, float * output, uint32_t delay_ms);

/// Set the relay driver action type.
bool tune_set_action_type(TuneId id, DriverAction type);

/// Set the target setpoint.
bool tune_set_setpoint(TuneId id, float setpoint);

/// Set max and min relay output steps.
bool tune_set_output_step(TuneId id, float max_step, float min_step);

/// Set the controller type (PI or PID).
bool tune_set_ctrl_type(TuneId id, ControllerType type);

/// Get the computed Kp gain.
/// @return Kp value (also written to *kp).
float tune_get_kp(TuneId id, float * kp);

/// Get the computed Ki gain.
/// @return Ki value (also written to *ki).
float tune_get_ki(TuneId id, float * ki);

/// Get the computed Kd gain.
/// @return Kd value (also written to *kd).
float tune_get_kd(TuneId id, float * kd);

/// Get all computed PID gains at once.
/// @return true on success.
bool tune_get_pid(TuneId id, float * kp, float * ki, float * kd);

/// Get the current autotune state.
/// @return true on success.
bool tune_get_state(TuneId id, TuneState * state);

}  // namespace drone
