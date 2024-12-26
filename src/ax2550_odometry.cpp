/* Odometrie for a 2 wheeled robot based on absolute values of
*  encoder values
*/

#include "ax2550_driver/ax2550_odometry.hpp"
#include <cmath>
#include <climits>

namespace ax2550_driver
{

Odometry::Odometry(
  double wheel_diameter, double wheel_base_length, double encoder_resolution,
  rclcpp::Time start)
: enc_left_old_(0), enc_right_old_(), time_old_(start),
  wheel_radius_(wheel_diameter / 2), wheel_separation_(wheel_base_length)
{
  ticks_per_meter_ = encoder_resolution / (2 * wheel_radius_ * M_PI);
}

void
Odometry::init(rclcpp::Time start)
{
  // current position
  x = 0;
  y = 0;
  theta = 0;
  v_x = 0;
  v_y = 0;
  //current speed
  v_linear = 0;
  v_angular = 0;

  // encoder values last time called
  time_old_ = start;
  enc_left_old_ = 0;
  enc_right_old_ = 0;
}

/* calculate Odometrie changes based on absolute changes of
*  encoder values and time
*/
void
Odometry::update(long enc_left, long enc_right, rclcpp::Time time)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("Odometry::update"),
    "enc_left: %lu, enc_right: %lu, time: %f", enc_right, enc_left, time.seconds());
  if (enc_left == LONG_MAX || enc_left == 0 || enc_right == LONG_MAX || enc_right == 0 ) {
    // ignore these invalid encoder values
    return;
  }

  // time difference
  double dt = (time - this->time_old_).seconds();
  this->time_old_ = time;
  // movement difference by wheel
  double dright = (enc_right - this->enc_right_old_) / this->ticks_per_meter_;
  double dleft = (enc_left - this->enc_left_old_) / this->ticks_per_meter_;
  this->enc_right_old_ = enc_right;
  this->enc_left_old_ = enc_left;
  if (dright == 0 && dleft == 0)
  {
    this->v_linear = 0.0;
    this->v_angular = 0.0;
    this->v_x = 0.0;
    this->v_y = 0.0;
    return;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("Odometry::update"),
    "dright: %f, dleft: %f, ticks_per_meter_: %f", dright, dleft, this->ticks_per_meter_);

  // linear and angular differences
  double dxy_ave = (dright + dleft) / 2.0;
  double dth = (dright - dleft) / this->wheel_separation_;
  // linear and angular speed
  double vxy = dxy_ave / dt;
  double vth = dth / dt;
  this->v_linear = vxy;
  this->v_angular = vth;
  RCLCPP_DEBUG(
    rclcpp::get_logger("Odometry::update"),
    "dxy_ave: %f, dth: %f, vxy: %f, vth: %f", dxy_ave, dth, vxy, vth);

  if (dxy_ave != 0) {
    // position change
    double dx = std::cos(dth) * dxy_ave;
    double dy = -std::sin(dth) * dxy_ave;
    // speed
    v_x = dx / dt;
    v_y = dy / dt;
    // integrate new position
    this->x += (std::cos(this->theta) * dx - std::sin(this->theta) * dy);
    this->y += (std::sin(this->theta) * dx + std::cos(this->theta) * dy);
    RCLCPP_DEBUG(
      rclcpp::get_logger("Odometry::update"),
      "dx: %f, dy: %f, x: %f, y: %f", dx, dy, this->x, this->y);
  }
  if (dth != 0) {
    // new angle
    this->theta += dth;
  }

}

void
Odometry::get_messages(
  nav_msgs::msg::Odometry & odom, geometry_msgs::msg::TransformStamped & transform,
  rclcpp::Time time,
  std::string odom_frame_id, std::string base_frame_id)
{
  odom.header.stamp = time;
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = base_frame_id;
  // Set the odom position and orientation
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  odom.pose.pose.orientation = tf2::toMsg(q);
  // transformation (tf)
  transform.header.stamp = odom.header.stamp;
  transform.header.frame_id = odom_frame_id;
  transform.child_frame_id = base_frame_id;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = odom.pose.pose.orientation;
  // Set the velocities
  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = v_y;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = v_angular;

  odom.pose.covariance[0] = 0.00001;
  odom.pose.covariance[7] = 0.00001;
  odom.pose.covariance[14] = 1000000000000.0;
  odom.pose.covariance[21] = 1000000000000.0;
  odom.pose.covariance[28] = 1000000000000.0;
  odom.pose.covariance[35] = 0.001;

}

} // namespace ax2550_driver
