// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file fuzzy_pid.cpp
/// @brief Fuzzy logic PID gain scheduler. Evaluates membership functions on
///        error and delta-error, applies fuzzy inference rules, and outputs
///        delta-Kp/Ki/Kd adjustments to adapt PID gains online.

#include "drone/control/fuzzy_pid.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone
{

namespace
{

float clamp(float value, float lo, float hi)
{
  return std::clamp(value, lo, hi);
}

// Membership functions: map a crisp input to a fuzzy membership value [0, 1].
// Each function implements a different shape (Gaussian, generalized bell,
// sigmoid, trapezoidal, triangular, Z-shaped).
float gaussmf(float x, float sigma, float c)
{
  return std::exp(-std::pow((x - c) / sigma, 2.0f));
}

float gbellmf(float x, float a, float b, float c)
{
  return 1.0f / (1.0f + std::pow(std::fabs((x - c) / a), 2.0f * b));
}

float sigmf(float x, float a, float c)
{
  return 1.0f / (1.0f + std::exp(a * (c - x)));
}

float trapmf(float x, float a, float b, float c, float d)
{
  if (x >= a && x < b) return (x - a) / (b - a);
  if (x >= b && x < c) return 1.0f;
  if (x >= c && x <= d) return (d - x) / (d - c);
  return 0.0f;
}

float trimf(float x, float a, float b, float c)
{
  return trapmf(x, a, b, b, c);
}

float zmf(float x, float a, float b)
{
  if (x <= a) return 1.0f;
  if (x <= (a + b) / 2.0f) return 1.0f - 2.0f * std::pow((x - a) / (b - a), 2.0f);
  if (x < b) return 2.0f * std::pow((x - b) / (b - a), 2.0f);
  return 0.0f;
}

float mf(float x, unsigned int mf_type, const float * params)
{
  switch (mf_type) {
    case 0:
      return gaussmf(x, params[0], params[1]);
    case 1:
      return gbellmf(x, params[0], params[1], params[2]);
    case 2:
      return sigmf(x, params[0], params[2]);
    case 3:
      return trapmf(x, params[0], params[1], params[2], params[3]);
    case 5:
      return zmf(x, params[0], params[1]);
    default:
      return trimf(x, params[0], params[1], params[2]);
  }
}

float fuzzy_or(float a, float b, unsigned int type)
{
  if (type == 1) return a + b - a * b;
  if (type == 2) return std::fmin(1.0f, a + b);
  return std::fmax(a, b);
}

float fuzzy_and(float a, float b, unsigned int type)
{
  if (type == 1) return a * b;
  if (type == 2) return std::fmax(0.0f, a + b - 1.0f);
  return std::fmin(a, b);
}

float fuzzy_equilibrium(float a, float b, float p)
{
  return std::pow(a * b, 1.0f - p) * std::pow(1.0f - (1.0f - a) * (1.0f - b), p);
}

float fo(float a, float b, unsigned int type)
{
  if (type < 3) return fuzzy_and(a, b, type);
  if (type < 6) return fuzzy_or(a, b, type - 3);
  return fuzzy_equilibrium(a, b, 0.5f);
}

}  // namespace

FuzzyPID::FuzzyPID()
{
  float rule_base[][kQfDefault] = {
    {PB, PB, PM, PM, PS, ZO, ZO}, {PB, PB, PM, PS, PS, ZO, NS}, {PM, PM, PM, PS, ZO, NS, NS},
    {PM, PM, PS, ZO, NS, NM, NM}, {PS, PS, ZO, NS, NS, NM, NM}, {PS, ZO, NS, NM, NM, NM, NB},
    {ZO, ZO, NM, NM, NM, NB, NB}, {NB, NB, NM, NM, NS, ZO, ZO}, {NB, NB, NM, NS, NS, ZO, ZO},
    {NB, NM, NS, NS, ZO, PS, PS}, {NM, NM, NS, ZO, PS, PM, PM}, {NM, NS, ZO, PS, PS, PM, PB},
    {ZO, ZO, PS, PS, PM, PB, PB}, {ZO, ZO, PS, PM, PM, PB, PB}, {PS, NS, NB, NB, NB, NM, PS},
    {PS, NS, NB, NM, NM, NS, ZO}, {ZO, NS, NM, NM, NS, NS, ZO}, {ZO, NS, NS, NS, NS, NS, ZO},
    {ZO, ZO, ZO, ZO, ZO, ZO, ZO}, {PB, PS, PS, PS, PS, PS, PB}, {PB, PM, PM, PM, PS, PS, PB},
  };

  float mf_params[4 * kQfDefault] = {
    -3, -3, -2, 0, -3, -2, -1, 0, -2, -1, 0, 0, -1, 0, 1, 0, 0, 1, 2, 0, 1, 2, 3, 0, 2, 3, 3, 0,
  };

  Params params[8];
  for (auto & p : params) {
    p = {4, 1, 0, mf_params, rule_base, 100, 100, 8};
  }
  init(params);
}

FuzzyPID::FuzzyPID(const Params * fuzzy_params)
{
  init(fuzzy_params);
}

void FuzzyPID::init(const Params * fuzzy_params)
{
  const auto & p = fuzzy_params[0];
  kp_ = 1;
  ki_ = 1;
  kd_ = 1;
  delta_kp_max_ = delta_ki_max_ = delta_kd_max_ = 0;
  delta_kp_ = delta_ki_ = delta_kd_ = 0;

  error_max_.resize(p.control_id_count);
  delta_error_max_.resize(p.control_id_count);
  for (int i = 0; i < p.control_id_count; ++i) {
    error_max_[i] = fuzzy_params[i].max_error;
    delta_error_max_[i] = fuzzy_params[i].max_delta_error;
  }

  int output_count = 1;
  if (ki_ > 1e-4f) {
    output_count += 1;
    if (kd_ > 1e-4f) output_count += 1;
  }

  fuzzy_input_num_ = 2;
  fuzzy_output_num_ = output_count;
  fo_type_ = p.fo_type;

  mf_type_.assign(fuzzy_input_num_ + fuzzy_output_num_, p.mf_type);
  fuzzy_output_.assign(fuzzy_output_num_, 0.0f);
  mf_params_.assign(p.mf_params, p.mf_params + 4 * kQfDefault);

  rule_base_.resize(fuzzy_output_num_ * kQfDefault * kQfDefault);
  for (unsigned int k = 0; k < fuzzy_output_num_ * kQfDefault; ++k) {
    for (int i = 0; i < kQfDefault; ++i) {
      rule_base_[k * kQfDefault + i] = p.rule_base[k][i];
    }
  }

  df_type_ = p.df_type;
  control_id_count_ = p.control_id_count;
  last_error_.assign(p.control_id_count, 0.0f);
  current_error_ = 0;
}

/// Mean-of-centers defuzzification: compute weighted average of rule consequents
/// using joint membership values as weights.
void FuzzyPID::moc(
  const float * joint_membership, const unsigned int * index, const unsigned int * count)
{
  float denominator = 0;
  std::vector<float> numerator(fuzzy_output_num_, 0.0f);

  for (unsigned int i = 0; i < count[0]; ++i)
    for (unsigned int j = 0; j < count[1]; ++j) denominator += joint_membership[i * count[1] + j];

  for (unsigned int k = 0; k < fuzzy_output_num_; ++k)
    for (unsigned int i = 0; i < count[0]; ++i)
      for (unsigned int j = 0; j < count[1]; ++j)
        numerator[k] +=
          joint_membership[i * count[1] + j] *
          rule_base_[k * kQfDefault * kQfDefault + index[i] * kQfDefault + index[count[0] + j]];

  for (unsigned int l = 0; l < fuzzy_output_num_; ++l)
    fuzzy_output_[l] = numerator[l] / denominator;
}

void FuzzyPID::df(
  const float * joint_membership, const unsigned int * output, const unsigned int * count,
  int /*df_type*/)
{
  moc(joint_membership, output, count);
}

/// Core fuzzy inference: fuzzify inputs (error, delta-error), compute joint
/// membership via the configured fuzzy operator, evaluate the rule base, and
/// defuzzify to produce gain deltas (delta_kp_, delta_ki_, delta_kd_).
void FuzzyPID::fuzzy_control(float e, float de)
{
  ke_ = e;
  kde_ = de;

  float membership[kQfDefault * 2];
  unsigned int index[kQfDefault * 2];
  unsigned int count[2] = {0, 0};

  int j = 0;
  for (int i = 0; i < kQfDefault; ++i) {
    float temp = mf(e, mf_type_[0], mf_params_.data() + 4 * i);
    if (temp > 1e-4f) {
      membership[j] = temp;
      index[j++] = i;
    }
  }
  count[0] = j;

  for (int i = 0; i < kQfDefault; ++i) {
    float temp = mf(de, mf_type_[1], mf_params_.data() + 4 * i);
    if (temp > 1e-4f) {
      membership[j] = temp;
      index[j++] = i;
    }
  }
  count[1] = j - count[0];

  if (count[0] == 0 || count[1] == 0) {
    std::fill(fuzzy_output_.begin(), fuzzy_output_.end(), 0.0f);
    return;
  }

  std::vector<float> joint_membership(count[0] * count[1]);
  for (unsigned int i = 0; i < count[0]; ++i)
    for (unsigned int jj = 0; jj < count[1]; ++jj)
      joint_membership[i * count[1] + jj] = fo(membership[i], membership[count[0] + jj], fo_type_);

  df(joint_membership.data(), index, count, 0);

  delta_kp_ = fuzzy_output_[0] / 3.0f * delta_kp_max_;
  delta_ki_ = (fuzzy_output_num_ >= 2) ? fuzzy_output_[1] / 3.0f * delta_ki_max_ : 0;
  delta_kd_ = (fuzzy_output_num_ >= 3) ? fuzzy_output_[2] / 3.0f * delta_kd_max_ : 0;
}

/// High-level API: compute error and delta-error for a given control axis,
/// normalize them by the per-axis max ranges, run fuzzy inference, and
/// add the resulting gain deltas to kp/ki/kd (modified in place).
void FuzzyPID::fuzzy_pid_control(
  float real, float idea, int control_id, float & kp, float & ki, float & kd, float delta_k)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  if (delta_k != 0.0f) {
    delta_kp_max_ = kp / delta_k;
    delta_ki_max_ = ki / delta_k;
    delta_kd_max_ = kd / delta_k;
  }

  if (control_id >= control_id_count_ || control_id < 0) return;

  last_error_[control_id] = current_error_;
  current_error_ = idea - real;
  float delta_error = current_error_ - last_error_[control_id];

  delta_error = clamp(delta_error, -delta_error_max_[control_id], delta_error_max_[control_id]);
  current_error_ = clamp(current_error_, -error_max_[control_id], error_max_[control_id]);

  fuzzy_control(
    current_error_ / error_max_[control_id] * 3.0f,
    delta_error / delta_error_max_[control_id] * 3.0f);

  kp += delta_kp_;
  ki += delta_ki_;
  kd += delta_kd_;
}

void FuzzyPID::set_rule_base(int i, int j, float value)
{
  rule_base_[i * kQfDefault * kQfDefault + j] = value;
}

}  // namespace drone
