#include "drone/math/pid.hpp"

#include "drone/math/utils.hpp"

#include <cmath>
#include <cstdio>

namespace drone
{

PID::PID(
  std::string pid_name, float kp, float ki, float kd, float kff, float kdff, float kimax,
  float srmax)
: pid_name(std::move(pid_name)),
  _kp(kp),
  _ki(ki),
  _kd(kd),
  _kff(kff),
  _kdff(kdff),
  _kimax(kimax),
  srmax(srmax)
{
  _pid_info._kP = _kp;
  _pid_info._kI = _ki;
  _pid_info._kD = _kd;
  _pid_info.slew_rate = srmax;
}

PID::PID(float kp, float ki, float kd, float kff, float kdff, float kimax, float srmax)
: _kp(kp), _ki(ki), _kd(kd), _kff(kff), _kdff(kdff), _kimax(kimax), srmax(srmax)
{
  _pid_info._kP = _kp;
  _pid_info._kI = _ki;
  _pid_info._kD = _kd;
  _pid_info.slew_rate = srmax;
}

PID::PID(const Defaults & defaults)
: _kp(defaults.p),
  _ki(defaults.i),
  _kd(defaults.d),
  _kff(defaults.ff),
  _kdff(defaults.dff),
  _kimax(defaults.imax),
  srmax(defaults.srmax)
{
  _pid_info._kP = _kp;
  _pid_info._kI = _ki;
  _pid_info._kD = _kd;
  _pid_info.slew_rate = defaults.srmax;
}

void PID::set_gains(const Defaults & defaults)
{
  _pid_info._kP = defaults.p;
  _pid_info._kI = defaults.i;
  _pid_info._kD = defaults.d;
}

void PID::set_gains(float kp, float ki, float kd)
{
  _pid_info._kP = kp;
  _pid_info._kI = ki;
  _pid_info._kD = kd;
}

void PID::set_pid(const Defaults & defaults)
{
  _kp = defaults.p;
  _ki = defaults.i;
  _kd = defaults.d;
  _kff = defaults.ff;
  _kdff = defaults.dff;
  _kimax = defaults.imax;
}

void PID::set_pid(float kp, float ki, float kd)
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::get_pid(float & kp, float & ki, float & kd)
{
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

float PID::update_all(
  float measurement, float target, float dt, float limit, float velocity, bool use_increment)
{
  _current_time += dt;

  _pid_info.target = target;
  _pid_info.actual = measurement;
  _pid_info.output_max = limit;
  _error = target - measurement;

  if (use_increment) {
    _pid_info.last_last_error = _pid_info.last_error;
    _pid_info.last_error = _pid_info.error;
    _pid_info.error = _error;
  } else {
    _pid_info.error = _error;
  }

#ifdef fuzzy_pid_dead_zone
  float dead_zone = 100.0f;
  if (_error < dead_zone && _error > -dead_zone) {
    _error = 0;
  } else if (_error > dead_zone) {
    _error -= dead_zone;
  } else if (_error < -dead_zone) {
    _error += dead_zone;
  }
#endif

  if (use_increment) {
    _pid_info.P = (_error - _pid_info.last_error) * _pid_info._kP;
  } else {
    _pid_info.P = _error * _pid_info._kP;
  }

  update_i(dt, limit);

  if (std::isfinite(velocity)) {
    if (use_increment) {
      _derivative = ((_error - 2 * _pid_info.last_error + _pid_info.last_last_error) / dt);
    } else {
      _derivative = -velocity;
      _pid_info.D = _derivative * _pid_info._kD;
    }
  } else {
    _derivative = calculate_improved_derivative(_current_time, dt);
    if (!is_equal(_derivative, 0.0f, 0.0001f)) {
      _pid_info.D = _derivative * _pid_info._kD;
    } else {
      _pid_info.D = 0.0f;
    }
  }

  _pid_info.FF = target * _kff;
  _pid_info.DFF = _derivative * _kdff;
  _pid_info.output = _pid_info.P + _pid_info.I + _pid_info.D + _pid_info.FF + _pid_info.DFF;
  _pid_info.Dmod = _error;
  _pid_info.target = target;
  _pid_info.actual = measurement;
  _pid_info.reset = false;
  _pid_info.PD_limit = false;
  _pid_info.slew_rate = srmax;

  if (limit > 0) {
    if (_pid_info.output > limit) return limit;
    if (_pid_info.output < -limit) return -limit;
  }

  _pid_info.last_output = _pid_info.output;
  return _pid_info.output;
}

float PID::update_all_increment(float measurement, float target, float dt, float limit)
{
  _pid_info.target = target;
  _pid_info.actual = measurement;
  _pid_info.output_max = limit;
  _error = target - measurement;

  // Capture previous errors BEFORE overwriting (fix: issue #3 from review)
  _pid_info.last_last_error = _pid_info.last_error;
  _pid_info.last_error = _pid_info.error;
  _pid_info.error = _error;
  _pid_info.last_output = 0.0f;

#ifdef fuzzy_pid_dead_zone
  float dead_zone = 100.0f;
  if (_error < dead_zone && _error > -dead_zone) {
    _error = 0;
  } else if (_error > dead_zone) {
    _error -= dead_zone;
  } else if (_error < -dead_zone) {
    _error += dead_zone;
  }
#endif

  _pid_info.P = (_error - _pid_info.last_error) * _pid_info._kP;
  update_i(dt, limit);
  _derivative = ((_error - 2 * _pid_info.last_error + _pid_info.last_last_error) / dt);
  _pid_info.FF = target * _kff;
  _pid_info.DFF = _derivative * _kdff;
  _pid_info.output_increment =
    _pid_info.P + _pid_info.I + _pid_info.D + _pid_info.FF + _pid_info.DFF;
  _incr_output = _pid_info.last_output + _pid_info.output_increment;
  _pid_info.Dmod = _error;
  _pid_info.target = target;
  _pid_info.actual = measurement;
  _pid_info.reset = false;
  _pid_info.PD_limit = false;
  _pid_info.slew_rate = srmax;

  if (limit > 0) {
    if (_pid_info.output_increment > limit) return limit;
    if (_pid_info.output_increment < -limit) return -limit;
  }

  _pid_info.last_output = _incr_output;
  return _pid_info.output_increment = _incr_output;
}

void PID::update_i(float dt, float limit)
{
  if (is_zero(_pid_info._kI) || !is_positive(dt)) {
    _pid_info.I = 0.0f;
    _pid_info.limit = (limit > 0);
    return;
  }

  float integral_increment = _error * _pid_info._kI * dt;

  // Anti-windup: decay when error crosses zero
  if ((_pid_info.I >= 0 && _error <= 0) || (_pid_info.I <= 0 && _error >= 0)) {
    _pid_info.I *= 0.8f;
  }

  if (limit <= 0) {
    _pid_info.I += integral_increment;
    _pid_info.I = constrain_float(_pid_info.I, _kimax, -_kimax);
    _pid_info.limit = false;
    return;
  }

  float saturated_output = constrain_float(_pid_info.output, limit, -limit);
  bool is_saturated = (std::fabs(_pid_info.output - saturated_output) > 1e-6f);

  if (is_saturated) {
    if ((_pid_info.output > limit && _error > 0) || (_pid_info.output < -limit && _error < 0)) {
      if ((_pid_info.I > 0 && _error < 0) || (_pid_info.I < 0 && _error > 0)) {
        _pid_info.I += integral_increment;
      } else {
        if (std::fabs(_pid_info.I) > _kimax * 0.9f) {
          _pid_info.I *= 0.97f;
        }
      }
    } else {
      _pid_info.I += integral_increment;
    }

    // Anti-windup feedback
    if (!is_zero(_pid_info._kP)) {
      float kb = _pid_info._kI / _pid_info._kP;
      float saturation_error = _pid_info.output - saturated_output;
      float anti_windup_correction = -saturation_error * kb * dt;
      _pid_info.I += anti_windup_correction;
    }
  } else {
    _pid_info.I += integral_increment;
  }

  _pid_info.I = constrain_float(_pid_info.I, _kimax, -_kimax);

  // Integral decay near zero error
  if (std::fabs(_error) < 0.05f && std::fabs(_pid_info.I) > 0.010f) {
    _pid_info.I *= 0.99f;
  }

  _pid_info.limit = (limit > 0);
}

void PID::print_update_info()
{
  std::printf(
    "PID%s:tar:%+10f mea:%+5f err:%+5f P:%+10f I:%+10f D:%+10f Out:%f _MAX:%f\n", pid_name.c_str(),
    _pid_info.target, _pid_info.actual, _pid_info.error, _pid_info.P, _pid_info.I, _pid_info.D,
    _pid_info.output, _pid_info.output_max);
}

void PID::reset_I()
{
  _pid_info.I = 0.0f;
  _integrator = 0.0f;
}

void PID::reset_all()
{
  _pid_info = PIDInfo{};
  _integrator = 0.0f;
  _error = 0.0f;
  _derivative = 0.0f;
  _history_index = 0;
  _history_full = false;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    _error_history[i] = 0.0f;
    _time_history[i] = 0.0f;
  }
}

float PID::get_integrator() const
{
  return _pid_info.I;
}

void PID::set_integrator(float integrator)
{
  _pid_info.I = constrain_float(integrator, _kimax, -_kimax);
  _integrator = _pid_info.I;
}

float PID::smooth_data(float current_value, float alpha)
{
  float smoothed_value = alpha * current_value + (1 - alpha) * _last_smoothed;
  _last_smoothed = smoothed_value;
  return smoothed_value;
}

float PID::calculate_improved_derivative(float current_time, float dt)
{
  _error_history[_history_index] = _error;
  _time_history[_history_index] = current_time;
  _history_index = (_history_index + 1) % HISTORY_SIZE;
  if (!_history_full && _history_index == 0) {
    _history_full = true;
  }

  if (!_history_full) {
    return -(_pid_info.Dmod - _error) / dt;
  }

  // Least-squares line fit for derivative
  float sum_time = 0.0f, sum_error = 0.0f;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum_time += _time_history[i];
    sum_error += _error_history[i];
  }
  float mean_time = sum_time / HISTORY_SIZE;
  float mean_error = sum_error / HISTORY_SIZE;

  float numerator = 0.0f, denominator = 0.0f;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    float time_diff = _time_history[i] - mean_time;
    float error_diff = _error_history[i] - mean_error;
    numerator += time_diff * error_diff;
    denominator += time_diff * time_diff;
  }

  if (std::fabs(denominator) < 1e-6f) {
    return -(_pid_info.Dmod - _error) / dt;
  }

  return -(numerator / denominator);
}

}  // namespace drone
