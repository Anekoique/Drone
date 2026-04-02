#pragma once

#include "drone/utils/readyaml.hpp"

#include <cfloat>
#include <string>

#define DEFAULT_VELOCITY INFINITY

namespace drone
{

class PID
{
public:
  struct Defaults
  {
    float p = 0;
    float i = 0;
    float d = 0;
    float ff = 0;
    float dff = 0;
    float imax = 10;
    float filt_T_hz = 0;
    float filt_E_hz = 0;
    float filt_D_hz = 0;
    float srmax = 0;
    float srtau = 0;
  };

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

  void set_gains(const Defaults & defaults);
  void set_gains(float kp, float ki, float kd);
  void set_pid(float kp, float ki, float kd);
  void set_pid(const Defaults & defaults);
  void get_pid(float & kp, float & ki, float & kd);

  float update_all(
    float measurement, float target, float dt, float limit, float velocity = DEFAULT_VELOCITY,
    bool use_increment = false);
  float update_all_increment(float measurement, float target, float dt, float limit);
  void update_i(float dt, float limit);
  void print_update_info();

  void reset_I();
  void reset_all();
  float get_integrator() const;
  void set_integrator(float integrator);

  float smooth_data(float current_value, float alpha);
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
    float P = 0;
    float I = 0;
    float D = 0;
    float FF = 0;
    float DFF = 0;
    float output = 0;
    float last_output = 0;
    float output_max = 0;
    float output_increment = 0;
    float Dmod = 0;
    float slew_rate = 0;
    bool limit = false;
    bool PD_limit = false;
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
