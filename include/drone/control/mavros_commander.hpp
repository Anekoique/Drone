#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <string>

namespace drone::control
{

/// Encapsulates all MAVROS setpoint publishing.
/// Ports all 6 legacy publishers from PosControl.
class MavrosCommander
{
public:
  MavrosCommander(rclcpp::Node & node, const std::string & ns);
  MavrosCommander() = delete;

  void send_velocity(const Eigen::Vector4f & vel);
  void send_position(double x, double y, double z, double yaw);
  void send_position(const Eigen::Vector4f & pos_yaw);
  void send_acceleration(const Eigen::Vector4f & accel);
  void send_attitude(const Eigen::Vector4f & attitude_thrust);
  void publish_setpoint_raw(const Eigen::Vector4f & pos, const Eigen::Vector4f & vel);
  void publish_setpoint_raw_global(double lat, double lon, double alt, float yaw);

  /// Timed velocity: sends vel for duration seconds, returns true when done.
  /// Returns true and resets timed_active_ when duration has elapsed.
  /// A subsequent call begins a new timed command.
  bool send_velocity_timed(const Eigen::Vector4f & vel, double duration_sec);

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr raw_global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;

  // State for send_velocity_timed (replaces static locals)
  rclcpp::Time timed_start_{0, 0, RCL_ROS_TIME};
  bool timed_active_ = false;
};

}  // namespace drone::control
