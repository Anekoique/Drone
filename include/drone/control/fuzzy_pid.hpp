// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

namespace drone
{

/// Fuzzy quantity field sizes for membership function discretization.
enum QuantityFields : int
{
  kQfSmall = 5,
  kQfMiddle = 7,
  kQfLarge = 8,
};

/// Default fuzzy quantity field size.
constexpr int kQfDefault = kQfMiddle;

// Fuzzy linguistic variable values
// NOLINTBEGIN(readability-identifier-naming)
constexpr int NB = -3;  ///< Negative Big
constexpr int NM = -2;  ///< Negative Medium
constexpr int NS = -1;  ///< Negative Small
constexpr int ZO = 0;   ///< Zero
constexpr int PS = 1;   ///< Positive Small
constexpr int PM = 2;   ///< Positive Medium
constexpr int PB = 3;   ///< Positive Big
// NOLINTEND(readability-identifier-naming)

/// Fuzzy PID controller. Adjusts PID gains online using fuzzy inference rules.
class FuzzyPID
{
public:
  /// Configuration for a single fuzzy PID axis.
  struct Params
  {
    unsigned int mf_type;                  ///< Membership function type
    unsigned int fo_type;                  ///< Fuzzy output type
    unsigned int df_type;                  ///< Defuzzification type
    const float * mf_params;               ///< Membership function parameters
    const float (*rule_base)[kQfDefault];  ///< Fuzzy rule table
    float max_error;                       ///< Error universe of discourse
    float max_delta_error;                 ///< Delta-error universe of discourse
    int control_id_count;                  ///< Number of controllers sharing this config
  };

  FuzzyPID();

  /// Construct and initialize with fuzzy parameters.
  /// @param fuzzy_params Array of Params, one per controller axis.
  explicit FuzzyPID(const Params * fuzzy_params);
  ~FuzzyPID() = default;

  /// Initialize or reinitialize with new parameters.
  /// @param fuzzy_params Array of Params, one per controller axis.
  void init(const Params * fuzzy_params);

  /// Run fuzzy inference for given error and delta-error.
  /// @param e Current error.
  /// @param de Current delta-error (error derivative).
  void fuzzy_control(float e, float de);

  /// Compute adapted PID gains for a specific control axis.
  /// @param real Current measurement.
  /// @param idea Desired setpoint.
  /// @param control_id Controller axis index.
  /// @param kp Output proportional gain.
  /// @param ki Output integral gain.
  /// @param kd Output derivative gain.
  /// @param delta_k Gain adjustment scale factor.
  void fuzzy_pid_control(
    float real, float idea, int control_id, float & kp, float & ki, float & kd,
    float delta_k = 2.0f);

  /// Override a single rule in the fuzzy rule table.
  /// @param i Row index (error level).
  /// @param j Column index (delta-error level).
  /// @param value New rule output value.
  void set_rule_base(int i, int j, float value);

private:
  int control_id_count_ = 0;
  float ke_ = 0;
  float kde_ = 0;

  unsigned int fuzzy_input_num_ = 0;
  unsigned int fuzzy_output_num_ = 0;
  unsigned int fo_type_ = 0;
  unsigned int df_type_ = 0;
  std::vector<unsigned int> mf_type_;
  std::vector<float> mf_params_;
  std::vector<float> rule_base_;
  std::vector<float> fuzzy_output_;

  float kp_ = 0;
  float ki_ = 0;
  float kd_ = 0;
  float delta_kp_max_ = 0;
  float delta_ki_max_ = 0;
  float delta_kd_max_ = 0;
  float delta_kp_ = 0;
  float delta_ki_ = 0;
  float delta_kd_ = 0;

  std::vector<float> last_error_;
  float current_error_ = 0;
  std::vector<float> error_max_;
  std::vector<float> delta_error_max_;

  /// Mean of maxima defuzzification.
  void moc(const float * joint_membership, const unsigned int * index, const unsigned int * count);

  /// General defuzzification method.
  void df(
    const float * joint_membership, const unsigned int * output, const unsigned int * count,
    int df_type);
};

}  // namespace drone
