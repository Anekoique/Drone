#pragma once

#include <vector>

namespace drone
{

// Fuzzy quantity field sizes
enum QuantityFields : int
{
  kQfSmall = 5,
  kQfMiddle = 7,
  kQfLarge = 8,
};

constexpr int kQfDefault = kQfMiddle;

// Fuzzy linguistic variable values
// NOLINTBEGIN(readability-identifier-naming)
constexpr int NB = -3;
constexpr int NM = -2;
constexpr int NS = -1;
constexpr int ZO = 0;
constexpr int PS = 1;
constexpr int PM = 2;
constexpr int PB = 3;
// NOLINTEND(readability-identifier-naming)

class FuzzyPID
{
public:
  struct Params
  {
    unsigned int mf_type;
    unsigned int fo_type;
    unsigned int df_type;
    const float * mf_params;
    const float (*rule_base)[kQfDefault];
    float max_error;
    float max_delta_error;
    int control_id_count;
  };

  FuzzyPID();
  explicit FuzzyPID(const Params * fuzzy_params);
  ~FuzzyPID() = default;

  void init(const Params * fuzzy_params);
  void fuzzy_control(float e, float de);
  void fuzzy_pid_control(
    float real, float idea, int control_id, float & kp, float & ki, float & kd,
    float delta_k = 2.0f);
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

  void moc(const float * joint_membership, const unsigned int * index, const unsigned int * count);
  void df(
    const float * joint_membership, const unsigned int * output, const unsigned int * count,
    int df_type);
};

}  // namespace drone
