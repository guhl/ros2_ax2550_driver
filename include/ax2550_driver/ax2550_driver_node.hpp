#ifndef AX2550__AX2550_NODE_HPP_
#define AX2550__AX2550_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ax2550_driver/ax2550_comms.hpp"
#include "ax2550_driver/ax2550_odometry.hpp"

namespace ax2550_driver
{

class Ax2550DriverNode : public rclcpp::Node
{
public:
  Ax2550DriverNode(const rclcpp::NodeOptions & options);
  virtual ~Ax2550DriverNode();
  void cmd_vel_callback(const geometry_msgs::msg::Twist & cmd_vel);
  void cmd_ax2550_callback(const std_msgs::msg::String & cmd_ax2550);

private:
  void get_params_();
  void encoder_timer_callback_();
  void publish_odometry_();

  std::unique_ptr<Ax2550Comms> ax2550_comms_{};
  // set by parameter
  std::string device_name_;
  double encoder_resolution_;
  std::string amps_limit_;
  uint max_a_;
  uint max_b_;
  double speed_constant_;
  double wheel_separation_;
  double wheel_diameter_;
  uint encoder_poll_rate_;
  // motor commands
  std::string cmd_dir_fw_;
  std::string cmd_dir_bw_;
  std::string cmd_dir_le_;
  std::string cmd_dir_ri_;
  // frame ids
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool enable_odom_tf_;

  double wheel_circumference_;

  rclcpp::TimerBase::SharedPtr encoder_timer_;
  // Publishers
  ///< Odometry data publisher
  std::shared_ptr<Odometry> odometry;     ///< Calculates odometry
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Subscribers
  /// Velocity commands subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  /// ax2550 commands subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_ax2550_;
};

} // namespace ax2550_driver

#endif
