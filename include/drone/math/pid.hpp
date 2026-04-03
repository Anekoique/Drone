// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/utils/readyaml.hpp"

#include <cfloat>
#include <string>

#define DEFAULT_VELOCITY INFINITY

namespace drone
{

/// Full-featured PID controller with feedforward, slew-rate limiting,
/// incremental output mode, and derivative filtering.
class PID
{
public:
  /// Default gain and filter parameters, loadable from YAML.
  struct Defaults
  {
    float p = 0;
    float i = 0;
    float d = 0;
    float ff = 0;         ///< Feedforward gain
    float dff = 0;        ///< Derivative feedforward gain
    float imax = 10;      ///< Integrator saturation limit
    float filt_T_hz = 0;  ///< Target filter frequency (Hz)
    float filt_E_hz = 0;  ///< Error filter frequency (Hz)
    float filt_D_hz = 0;  ///< Derivative filter frequency (Hz)
    float srmax = 0;      ///< Slew-rate limit
    float srtau = 0;      ///< Slew-rate time constant
  };

  /// Load PID gains from a YAML config file.
  /// @param filename YAML filename in the package config directory.
  /// @param pid_name Key under which PID gains are stored.
  /// @return Defaults struct with loaded values (missing keys use defaults).
  static Defaults readPIDParameters(const std::string & filename, const std::string & pid_name)
  {
    YAML::Node config = ConfigLoader::load(filename);
    Defaults d;
    try {
      d.p = config[pid_name]["p"].as<double>();
      d.i = config[pid_name]["i"].as<double>();
      d.d = config[pid_name]["d"].as<double>();
      d.ff = config[pid_name]["ff"].as<double>();
      d.dff = config[pid_name]["dff"].as<double>();
      d.imax = config[pid_name]["imax"].as<double>();
    } catch (const std::exception & e) {
      // allow partial configs
    }
    return d;
  }

  PID(
    std::string pid_name, float kp, float ki, float kd, float kff = 0, float kdff = 0,
    float kimax = 2, float srmax = 0);
  PID(
    float kp, float ki, float kd, float kff = 0, float kdff = 0, float kimax = 2, float srmax = 0);
  explicit PID(const Defaults & defaults);
  PID() = default;

  /// Set all gains from a Defaults struct.
  void set_gains(const Defaults & defaults);

  /// Set P, I, D gains only.
  void set_gains(float kp, float ki, float kd);

  /// Set P, I, D gains (alias for set_gains).
  void set_pid(float kp, float ki, float kd);

  /// Set P, I, D gains from a Defaults struct (alias for set_gains).
  void set_pid(const Defaults & defaults);

  /// Read back current P, I, D gains.
  void get_pid(float & kp, float & ki, float & kd);

  /// Run a full PID update cycle.
  /// @param measurement Current measured value.
  /// @param target Desired setpoint.
  /// @param dt Time step in seconds.
  /// @param limit Output saturation limit.
  /// @param velocity Optional velocity feedback for derivative feedforward.
  /// @param use_increment If true, use incremental output mode.
  /// @return Controller output.
  float update_all(
    float measurement, float target, float dt, float limit, float velocity = DEFAULT_VELOCITY,
    bool use_increment = false);

  /// Incremental PID update (output is a delta from previous output).
  /// @return Incremental output change.
  float update_all_increment(float measurement, float target, float dt, float limit);

  /// Update only the integrator term.
  void update_i(float dt, float limit);

  /// Print diagnostic info for the last update.
  void print_update_info();

  /// Reset integrator to zero.
  void reset_I();

  /// Reset all internal state (integrator, history, derivative).
  void reset_all();

  /// @return Current integrator value.
  float get_integrator() const;

  /// @param integrator New integrator value.
  void set_integrator(float integrator);

  /// Exponential moving average smoother.
  /// @param current_value New sample.
  /// @param alpha Smoothing factor (0..1, lower = smoother).
  /// @return Smoothed value.
  float smooth_data(float current_value, float alpha);

  /// Compute derivative using error history (least-squares fit).
  /// @return Filtered derivative estimate.
  float calculate_improved_derivative(float current_time, float dt);

  float _integrator = 0;
  float _target = 0;
  float _error = 0;
  float _derivative = 0;
  std::string pid_name = "pid";

  static constexpr int HISTORY_SIZE = 5;
  float _error_history[HISTORY_SIZE] = {};
  float _time_history[HISTORY_SIZE] = {};
  int _history_index = 0;
  bool _history_full = false;

  /// Snapshot of the last PID computation for diagnostics.
  struct PIDInfo
  {
    float target = 0;
    float actual = 0;
    float error = 0;
    float last_error = 0;
    float last_last_error = 0;
    float _kP = 0;
    float _kI = 0;
    float _kD = 0;
    float P = 0;    ///< Proportional contribution
    float I = 0;    ///< Integral contribution
    float D = 0;    ///< Derivative contribution
    float FF = 0;   ///< Feedforward contribution
    float DFF = 0;  ///< Derivative feedforward contribution
    float output = 0;
    float last_output = 0;
    float output_max = 0;
    float output_increment = 0;
    float Dmod = 0;  ///< Derivative modifier from slew-rate limiter
    float slew_rate = 0;
    bool limit = false;     ///< True if output was saturated
    bool PD_limit = false;  ///< True if P+D alone exceeded limit
    bool reset = false;
    bool I_term_set = false;
  };
  PIDInfo _pid_info;

private:
  float _kp = 0, _ki = 0, _kd = 0;
  float _kff = 0;
  float _kdff = 0;
  float _kimax = 0;
  float filt_T_hz = 0;
  float filt_E_hz = 0;
  float filt_D_hz = 0;
  float srmax = 0;
  float srtau = 0;
  float _current_time = 0;
  float _incr_output = 0;
  float _last_smoothed = 0;
};

}  // namespace drone
