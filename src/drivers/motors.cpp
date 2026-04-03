#include "drone/drivers/motors.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace drone
{

Motors::Motors(rclcpp::Node & node, const std::string & mavros_ns) : node_(node)
{
  auto qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
  auto cb_group = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = cb_group;

  arm_client_ = node_.create_client<mavros_msgs::srv::CommandBool>(mavros_ns + "cmd/arming");
  mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(mavros_ns + "set_mode");
  home_client_ = node_.create_client<mavros_msgs::srv::CommandHome>(mavros_ns + "cmd/set_home");
  takeoff_client_ = node_.create_client<mavros_msgs::srv::CommandTOL>(mavros_ns + "cmd/takeoff");
  land_client_ = node_.create_client<mavros_msgs::srv::CommandTOL>(mavros_ns + "cmd/land");
  param_client_ = node_.create_client<mavros_msgs::srv::ParamSetV2>(mavros_ns + "param/set");

  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    mavros_ns + "state", qos, std::bind(&Motors::state_callback, this, std::placeholders::_1),
    sub_opt);

  home_sub_ = node_.create_subscription<mavros_msgs::msg::HomePosition>(
    mavros_ns + "home_position/home", qos,
    std::bind(&Motors::home_callback, this, std::placeholders::_1), sub_opt);
}

void Motors::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
  armed_ = msg->armed;
  connected_ = msg->connected;
  guided_ = msg->guided;
  mode_ = msg->mode;
  system_status_ = msg->system_status;
}

void Motors::home_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg)
{
  home_position_ = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
  home_position_global_ = Eigen::Vector3f(msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
  home_quaternion_ = Eigen::Quaternionf(
    msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

bool Motors::arm(bool do_arm)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = do_arm;
  auto future = arm_client_->async_send_request(req);
  return true;  // async — result checked via state callback
}

bool Motors::switch_mode(const std::string & mode)
{
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;
  auto future = mode_client_->async_send_request(req);
  return true;
}

bool Motors::set_home_position(float yaw)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
  req->current_gps = true;
  req->yaw = yaw;
  auto future = home_client_->async_send_request(req);
  return true;
}

bool Motors::command_takeoff_or_land(const std::string & cmd, float altitude, float yaw)
{
  auto client = (cmd == "TAKEOFF") ? takeoff_client_ : land_client_;
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->altitude = altitude;
  req->yaw = yaw;
  auto future = client->async_send_request(req);
  return true;
}

bool Motors::set_param(const std::string & name, double value)
{
  auto req = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
  req->param_id = name;
  req->value.double_value = value;
  req->value.type = 9;  // PARAM_TYPE_REAL64
  auto future = param_client_->async_send_request(req);
  return true;
}

bool Motors::takeoff(float /*current_z*/, float takeoff_altitude, float yaw)
{
  switch (takeoff_state_) {
    case TakeoffState::kInit:
      RCLCPP_INFO(node_.get_logger(), "Switching to GUIDED mode");
      switch_mode("GUIDED");
      takeoff_state_ = TakeoffState::kWaitForCommand;
      break;

    case TakeoffState::kWaitForCommand:
      if (takeoff_command) {
        RCLCPP_INFO(node_.get_logger(), "Arming motors");
        arm(true);
        takeoff_timer_.reset();
        takeoff_state_ = TakeoffState::kArmRequested;
      }
      break;

    case TakeoffState::kWaitForStable:
      if (takeoff_timer_.elapsed() > 1.0) {
        arm(true);
        takeoff_state_ = TakeoffState::kArmRequested;
      }
      break;

    case TakeoffState::kArmRequested:
      if (!armed_) {
        if (takeoff_timer_.elapsed() > 1.0) {
          arm(true);
          takeoff_timer_.reset();
        }
      } else {
        set_home_position(yaw);
        command_takeoff_or_land("TAKEOFF", takeoff_altitude * 2, yaw);
        takeoff_timer_.reset();
        takeoff_attempts_ = 1;
        takeoff_state_ = TakeoffState::kTakeoff;
      }
      break;

    case TakeoffState::kTakeoff:
      if (!armed_) {
        takeoff_state_ = TakeoffState::kArmRequested;
      } else if (system_status_ == 4) {  // MAV_STATE_ACTIVE
        RCLCPP_INFO(node_.get_logger(), "Takeoff successful");
        takeoff_state_ = TakeoffState::kEnd;
      } else if (takeoff_timer_.elapsed() > 2.0 && takeoff_attempts_ <= 5) {
        command_takeoff_or_land("TAKEOFF", takeoff_altitude * 2, yaw);
        takeoff_timer_.reset();
        takeoff_attempts_++;
      }
      break;

    case TakeoffState::kEnd:
      return true;
  }
  return false;
}

}  // namespace drone
