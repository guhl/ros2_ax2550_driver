#ifndef AX2550_DRIVER__AX2550_ODOMETRY_HPP_
#define AX2550_DRIVER__AX2550_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ax2550_driver
{

class Odometry
{
public:
  /*
  * wheel_diameter [m]
  * wheel_separation [m]
  * encoder_resolution [ticks per revolution]
  * start [time]
  */
  Odometry(
    double wheel_diameter, double wheel_separation, double encoder_resolution,
    rclcpp::Time start);

  void init(rclcpp::Time start);
  void update(long enc_left, long enc_right, rclcpp::Time time);
  void get_messages(
    nav_msgs::msg::Odometry & odom, geometry_msgs::msg::TransformStamped & transform,
    rclcpp::Time time,
    std::string odom_frame_id = "odom", std::string base_frame_id = "base_link");

  double x = 0.0;          ///< Position in x axis [m]
  double y = 0.0;          ///< Position in y axis [m]
  double theta = 0.0;      ///< Heading [rad]

  double v_x = 0.0;
  double v_y = 0.0;

  double v_linear = 0.0;    ///< Linear velocity [m/s]
  double v_angular = 0.0;    ///< Angular velocity [rad/s]

private:
  // Encoders values last time called
  long enc_left_old_;
  long enc_right_old_;
  rclcpp::Time time_old_;

  // Wheel kinematic parameters [m]:
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;

  // encoder translation
  double ticks_per_meter_;
};

} // namespace ax2550_driver

#endif // AX2550_DRIVER__AX2550_ODOMETRY_HPP_
