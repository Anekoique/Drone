#include "drone/control/autotune.hpp"

#include <cmath>
#include <cstdio>

namespace drone
{

namespace
{

constexpr int kLastPeakNum = 20;
constexpr int kMaxCycle = 200;
constexpr uint32_t kMaxTimeMs = 3600000;

enum class OffsetState : int
{
  kBelow = 0,
  kAbove
};

struct PeakValue
{
  float feedback = 0;
  uint32_t ms = 0;
};

struct PeakPair
{
  PeakValue max_peak;
  PeakValue min_peak;
};

struct TuneObject
{
  TuneState state = TuneState::kInit;
  ControllerType ctrl_type = ControllerType::kPID;
  DriverAction action_type = DriverAction::kPositive;
  TuneId id = -1;
  OffsetState offset = OffsetState::kAbove;
  float feedback = 0;
  float output = 0;
  float setpoint = 0;
  float max_output_step = 50;
  float min_output_step = 0;
  uint32_t timer = 0;
  uint32_t cycle_count = 0;
  uint32_t hysteresis_count = 5;
  uint32_t rise_hysteresis = 0;
  uint32_t fall_hysteresis = 0;
  uint32_t full_cycle_flag = 0;
  PeakPair peaks[kLastPeakNum]{};
  int32_t peak_write_pos = 0;
  float ku = 0, tu = 0, kp = 0, ki = 0, kd = 0;
  float amp_std_dev = 0;
  float cycle_std_dev = 0;
  float cur_amp_std_dev = 0;
  float cur_cycle_std_dev = 0;
};

TuneObject g_objects[kAutoTuneMaxObjects];

bool reset_defaults(TuneId id)
{
  if (id >= kAutoTuneMaxObjects) return false;
  auto & o = g_objects[id];
  o.id = -1;
  o.ctrl_type = ControllerType::kPID;
  o.max_output_step = 50;
  o.hysteresis_count = 5;
  o.action_type = DriverAction::kPositive;
  return true;
}

bool fsm_reset(TuneId id)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  auto & o = g_objects[id];
  o.state = TuneState::kInit;
  o.timer = 0;
  o.cycle_count = 0;
  o.rise_hysteresis = 0;
  o.fall_hysteresis = 0;
  o.offset = OffsetState::kAbove;
  o.peak_write_pos = 0;
  return true;
}

float calc_std_dev(float * data, uint32_t len)
{
  if (!data || len == 0) return 0;
  float avg = 0;
  for (uint32_t i = 0; i < len; i++) avg += data[i];
  avg /= static_cast<float>(len);
  float var = 0;
  for (uint32_t i = 0; i < len; i++) var += std::pow(data[i] - avg, 2.0f);
  return std::sqrt(var / static_cast<float>(len));
}

void peak_reset(TuneId id, int32_t ch, float sp)
{
  auto & p = g_objects[id].peaks[ch];
  p.max_peak = {sp, 0};
  p.min_peak = {sp, 0};
}

bool calc_pid(TuneId id)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  auto & o = g_objects[id];
  auto prev = (o.peak_write_pos == 0) ? kLastPeakNum - 1 : o.peak_write_pos - 1;
  auto & cur = o.peaks[o.peak_write_pos];
  auto & prv = o.peaks[prev];

  o.ku = 4.0f * (o.max_output_step - o.min_output_step) /
         ((cur.max_peak.feedback - cur.min_peak.feedback) * 3.14159f);
  o.tu = static_cast<float>(cur.max_peak.ms - prv.max_peak.ms) / 1000.0f;
  o.kp = (o.ctrl_type == ControllerType::kPID) ? 0.6f * o.ku : 0.8f * o.ku;
  o.ki = 0.5f * o.tu;
  o.kd = 0.125f * o.tu;
  return true;
}

}  // namespace

void tune_init()
{
  for (int i = 0; i < kAutoTuneMaxObjects; i++) reset_defaults(i);
}

TuneId tune_new(const TuneConfig * config)
{
  for (int i = 0; i < kAutoTuneMaxObjects; i++) {
    if (g_objects[i].id < 0) {
      g_objects[i].id = i;
      if (!config) return i;
      g_objects[i].ctrl_type = config->ctrl_type;
      g_objects[i].max_output_step = config->max_output_step;
      g_objects[i].min_output_step = config->min_output_step;
      g_objects[i].hysteresis_count = config->hysteresis_count;
      g_objects[i].setpoint = config->setpoint;
      g_objects[i].action_type = config->action_type;
      g_objects[i].amp_std_dev = config->amp_std_deviation;
      g_objects[i].cycle_std_dev = config->cycle_std_deviation;
      return i;
    }
  }
  return -1;
}

bool tune_release(TuneId id)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  auto & o = g_objects[id];
  if (
    o.state == TuneState::kFail || o.state == TuneState::kSuccess || o.state == TuneState::kInit) {
    reset_defaults(id);
    return true;
  }
  return false;
}

TuneState tune_work(TuneId id, float feedback, float * output, uint32_t delay_ms)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return TuneState::kFail;

  auto & o = g_objects[id];
  o.feedback = feedback;
  o.timer += delay_ms;

  float out_high =
    (o.action_type == DriverAction::kPositive) ? o.max_output_step : o.min_output_step;
  float out_low =
    (o.action_type == DriverAction::kPositive) ? o.min_output_step : o.max_output_step;

  switch (o.state) {
    case TuneState::kInit:
      fsm_reset(id);
      *output = (feedback <= o.setpoint) ? out_high : out_low;
      o.state = TuneState::kStartPoint;
      return TuneState::kInit;

    case TuneState::kStartPoint:
      if (feedback < o.setpoint) {
        o.rise_hysteresis = 0;
        if (++o.fall_hysteresis >= o.hysteresis_count) *output = out_high;
      } else {
        o.fall_hysteresis = 0;
        if (++o.rise_hysteresis >= o.hysteresis_count) {
          peak_reset(id, 0, o.setpoint);
          peak_reset(id, 1, o.setpoint);
          peak_reset(id, 2, o.setpoint);
          o.state = TuneState::kRunning;
        }
      }
      return TuneState::kInit;

    case TuneState::kRunning:
      if (feedback > o.setpoint) {
        o.fall_hysteresis = 0;
        if (++o.rise_hysteresis >= o.hysteresis_count) {
          *output = out_low;
          if (o.offset == OffsetState::kBelow) {
            o.offset = OffsetState::kAbove;
            o.full_cycle_flag = 0;
            o.peak_write_pos = (o.peak_write_pos >= kLastPeakNum - 1) ? 0 : o.peak_write_pos + 1;
            peak_reset(id, o.peak_write_pos, o.setpoint);
          }
          auto & pk = o.peaks[o.peak_write_pos];
          if (feedback >= pk.max_peak.feedback) {
            pk.max_peak = {feedback, o.timer};
          }
        }
      } else {
        o.rise_hysteresis = 0;
        if (++o.fall_hysteresis >= o.hysteresis_count) {
          *output = out_high;
          if (o.offset == OffsetState::kAbove) {
            o.offset = OffsetState::kBelow;
            o.cycle_count++;
          }
          auto & pk = o.peaks[o.peak_write_pos];
          if (feedback <= pk.min_peak.feedback) {
            pk.min_peak = {feedback, o.timer};
            o.full_cycle_flag = 1;
          } else if (o.full_cycle_flag > 0) {
            o.full_cycle_flag++;
          }

          if (o.cycle_count >= kLastPeakNum && o.full_cycle_flag >= o.hysteresis_count) {
            float peaks_arr[kLastPeakNum];
            float times[kLastPeakNum];
            for (int i = 0; i < kLastPeakNum; i++) {
              peaks_arr[i] = o.peaks[i].max_peak.feedback - o.peaks[i].min_peak.feedback;
              times[i] = static_cast<float>(o.peaks[i].min_peak.ms - o.peaks[i].max_peak.ms);
            }
            o.cur_amp_std_dev = calc_std_dev(peaks_arr, kLastPeakNum);
            o.cur_cycle_std_dev = calc_std_dev(times, kLastPeakNum);

            if (o.cur_amp_std_dev < o.amp_std_dev && o.cur_cycle_std_dev < o.cycle_std_dev) {
              o.state = TuneState::kSuccess;
            }
          }
        }
      }
      if (o.cycle_count > kMaxCycle || o.timer > kMaxTimeMs) {
        o.state = TuneState::kFail;
      }
      return TuneState::kRunning;

    case TuneState::kFail:
      calc_pid(id);
      fsm_reset(id);
      *output = out_low;
      return TuneState::kFail;

    case TuneState::kSuccess:
      calc_pid(id);
      fsm_reset(id);
      *output = out_low;
      return TuneState::kSuccess;

    default:
      return TuneState::kInit;
  }
}

bool tune_set_action_type(TuneId id, DriverAction type)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  g_objects[id].action_type = type;
  return true;
}

bool tune_set_setpoint(TuneId id, float setpoint)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  g_objects[id].setpoint = setpoint;
  return true;
}

bool tune_set_output_step(TuneId id, float max_step, float min_step)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  g_objects[id].max_output_step = max_step;
  g_objects[id].min_output_step = min_step;
  return true;
}

bool tune_set_ctrl_type(TuneId id, ControllerType type)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  g_objects[id].ctrl_type = type;
  return true;
}

float tune_get_kp(TuneId id, float * kp)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return 0;
  *kp = g_objects[id].kp;
  return 1;
}

float tune_get_ki(TuneId id, float * ki)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return 0;
  *ki = g_objects[id].ki;
  return 1;
}

float tune_get_kd(TuneId id, float * kd)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return 0;
  *kd = g_objects[id].kd;
  return 1;
}

bool tune_get_pid(TuneId id, float * kp, float * ki, float * kd)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  if (kp) tune_get_kp(id, kp);
  if (ki) tune_get_ki(id, ki);
  if (kd) tune_get_kd(id, kd);
  return true;
}

bool tune_get_state(TuneId id, TuneState * state)
{
  if (id >= kAutoTuneMaxObjects || g_objects[id].id < 0) return false;
  *state = g_objects[id].state;
  return true;
}

}  // namespace drone
