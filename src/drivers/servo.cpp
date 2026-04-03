#include "drone/drivers/servo.hpp"

#include <mavros_msgs/msg/command_code.hpp>

#include <chrono>

namespace drone
{

Servo::Servo(rclcpp::Node & node, const std::string & mavros_ns) : node_(node)
{
  client_ = node_.create_client<mavros_msgs::srv::CommandLong>(mavros_ns + "cmd/command");
}

bool Servo::set_servo(int servo_number, float pwm)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  req->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
  req->param1 = static_cast<float>(servo_number);
  req->param2 = pwm;

  RCLCPP_INFO(node_.get_logger(), "Setting servo %d to PWM %.0f", servo_number, pwm);

  auto future = client_->async_send_request(
    req, [this, servo_number](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture f) {
      try {
        auto reply = f.get();
        if (reply->success) {
          RCLCPP_INFO(node_.get_logger(), "Servo %d: success", servo_number);
        } else {
          RCLCPP_ERROR(node_.get_logger(), "Servo %d: failed", servo_number);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_.get_logger(), "Servo %d: exception: %s", servo_number, e.what());
      }
    });
  return true;
}

void Servo::fire_servo(int servo_id, float open_pwm, float close_pwm, double delay_sec)
{
  RCLCPP_INFO(
    node_.get_logger(), "Fire servo %d: open=%.0f close=%.0f delay=%.1fs", servo_id, open_pwm,
    close_pwm, delay_sec);
  set_servo(servo_id, open_pwm);

  // Store timer as member — prevents destruction before callback fires.
  // Per-servo map handles concurrent fire_servo calls for different IDs.
  pending_timers_[servo_id] = node_.create_wall_timer(
    std::chrono::duration<double>(delay_sec), [this, servo_id, close_pwm]() {
      set_servo(servo_id, close_pwm);
      pending_timers_.erase(servo_id);
    });
}

}  // namespace drone
