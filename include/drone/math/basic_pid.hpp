// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/utils/readyaml.hpp"

#include <algorithm>
#include <string>

namespace drone
{

/// Simple 3-axis PID controller. Computes velocity commands from position error.
/// Preserves legacy MYPID interface for mechanical migration.
class BasicPID
{
public:
  BasicPID() = default;

  /// Load PID gains and limits from a YAML config file.
  /// @param filename YAML filename in the package config directory.
  /// @param pid_name Key under which PID parameters are stored.
  void readPIDParameters(const std::string & filename, const std::string & pid_name)
  {
    try {
      YAML::Node config = ConfigLoader::load(filename);
      kp_ = config[pid_name]["p"].as<double>();
      ki_ = config[pid_name]["i"].as<double>();
      kd_ = config[pid_name]["d"].as<double>();
      output_limit_ = config[pid_name]["outputlimit"].as<double>();
      integral_limit = config[pid_name]["integrallimit"].as<double>();
    } catch (const std::exception &) {
      // Allow partial configs — use defaults for missing keys
    }
  }

  /// Read a single float value from a YAML config file.
  /// @param filename YAML filename in the package config directory.
  /// @param goal_name Top-level key to read.
  /// @return The float value.
  float read_goal(const std::string & filename, const std::string & goal_name)
  {
    YAML::Node config = ConfigLoader::load(filename);
    return config[goal_name].as<float>();
  }

  /// Compute velocity commands for 3 axes. Results stored in velocity_x/y/z.
  /// @param x_target Target X position.
  /// @param y_target Target Y position.
  /// @param z_target Target Z position.
  /// @param x_now Current X position.
  /// @param y_now Current Y position.
  /// @param z_now Current Z position.
  /// @param dt Time step in seconds.
  void Mypid(
    float x_target, float y_target, float z_target, float x_now, float y_now, float z_now, float dt)
  {
    velocity_x = -compute(x_target, x_now, dt);
    velocity_y = -compute(y_target, y_now, dt);
    velocity_z = compute(z_target, z_now, dt);
  }

  /// Compute a single-axis PID output.
  /// @param setpoint Desired value.
  /// @param measured Current value.
  /// @param dt Time step in seconds.
  /// @return Clamped control output.
  float compute(float setpoint, float measured, float dt)
  {
    float error = setpoint - measured;
    integral_ += error * dt;
    float derivative = (error - prev_error_) / dt;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = std::clamp(output, -output_limit_, output_limit_);
    integral_ = std::clamp(integral_, -integral_limit, integral_limit);

    prev_error_ = error;
    return output;
  }

  float velocity_x = 0, velocity_y = 0, velocity_z = 0;
  float kp_ = 0, ki_ = 0, kd_ = 0;
  float output_limit_ = 0;
  float integral_limit = 0;

private:
  float prev_error_ = 0;
  float integral_ = 0;
};

}  // namespace drone
