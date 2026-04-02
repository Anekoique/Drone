#pragma once

#include <cstdint>

namespace drone
{

constexpr int kAutoTuneMaxObjects = 4;

using TuneId = int32_t;

enum class ControllerType : int
{
  kPI,
  kPID,
};

enum class TuneState : int
{
  kInit = 0,
  kStartPoint,
  kRunning,
  kFail,
  kSuccess,
};

enum class DriverAction : int
{
  kPositive,
  kNegative,
};

struct TuneConfig
{
  ControllerType ctrl_type = ControllerType::kPID;
  DriverAction action_type = DriverAction::kPositive;
  float max_output_step = 50.0f;
  float min_output_step = 0.0f;
  uint32_t hysteresis_count = 5;
  float setpoint = 50.0f;
  float amp_std_deviation = 0.0f;
  float cycle_std_deviation = 0.0f;
};

void tune_init();
TuneId tune_new(const TuneConfig * config);
bool tune_release(TuneId id);
TuneState tune_work(TuneId id, float feedback, float * output, uint32_t delay_ms);
bool tune_set_action_type(TuneId id, DriverAction type);
bool tune_set_setpoint(TuneId id, float setpoint);
bool tune_set_output_step(TuneId id, float max_step, float min_step);
bool tune_set_ctrl_type(TuneId id, ControllerType type);
float tune_get_kp(TuneId id, float * kp);
float tune_get_ki(TuneId id, float * ki);
float tune_get_kd(TuneId id, float * kd);
bool tune_get_pid(TuneId id, float * kp, float * ki, float * kd);
bool tune_get_state(TuneId id, TuneState * state);

}  // namespace drone
