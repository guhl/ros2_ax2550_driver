#include <chrono>
#include "ax2550_driver/ax2550_driver_node.hpp"

namespace ax2550_driver
{

Ax2550DriverNode::Ax2550DriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ax2550_node", options), ax2550_comms_{new Ax2550Comms()},
  device_name_(std::string("")), encoder_resolution_(0.0),
  amps_limit_(std::string("")), max_a_(127), max_b_(127),
  speed_constant_(0.0),wheel_separation_(0.0), wheel_diameter_(0.0),
  encoder_poll_rate_(0),
  cmd_dir_fw_("!A"), cmd_dir_bw_("!a"), cmd_dir_le_("!B"), cmd_dir_ri_("!b"),
  wheel_circumference_(0.0)
{
  get_params_();
  wheel_circumference_ = wheel_diameter_ * M_PI;
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  if (this->enable_odom_tf_)
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Subscribers
  RCLCPP_INFO(this->get_logger(), "Subscribing to cmd_vel");
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&Ax2550DriverNode::cmd_vel_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribing to cmd_ax2550");
  sub_cmd_ax2550_ = this->create_subscription<std_msgs::msg::String>(
    "cmd_ax2550", 1, std::bind(&Ax2550DriverNode::cmd_ax2550_callback, this, std::placeholders::_1));
  // initialize the motor controller
  RCLCPP_INFO(this->get_logger(), "Initializing ax2550 on device %s", this->device_name_.c_str());
  this->ax2550_comms_->set_device_name(this->device_name_);
  this->ax2550_comms_->set_commands(
    this->cmd_dir_fw_, this->cmd_dir_bw_, this->cmd_dir_le_, this->cmd_dir_ri_);
  this->ax2550_comms_->init();
  if (this->amps_limit_ != ""){
    RCLCPP_INFO(this->get_logger(), "Setting amps limit to: %s", this->amps_limit_.c_str());
    this->ax2550_comms_->set_amps_limit(this->amps_limit_);
  }
  // reset encoders
  RCLCPP_INFO(this->get_logger(), "Resetting encoders");
  this->ax2550_comms_->reset_encoders();
  // reset odometry
  odometry.reset(
    new Odometry(
      wheel_diameter_, wheel_separation_, encoder_resolution_,
      this->now()));
  // start timer to update encoder based odometry
  std::chrono::milliseconds enc_ms{encoder_poll_rate_};
  this->encoder_timer_ =
    this->create_wall_timer(enc_ms, std::bind(&Ax2550DriverNode::encoder_timer_callback_, this));
}

Ax2550DriverNode::~Ax2550DriverNode()
{
}

void
Ax2550DriverNode::encoder_timer_callback_()
{
  long encoder1(0), encoder2(0);
  rclcpp::Time now = this->now();
  if (this->ax2550_comms_ != NULL && this->ax2550_comms_->is_connected()) {
    // query encoder ticks
    this->ax2550_comms_->query_encoders(encoder1, encoder2);
  } else {
    RCLCPP_ERROR(this->get_logger(), "cmd_vel_callback: ax2550 not connected");
  }
  // calculate updated odometry
  this->odometry->update(encoder1, encoder2, this->now());
  // publish odometry
  this->publish_odometry_();
}

void
Ax2550DriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist & cmd_vel)
{
  if (this->ax2550_comms_ == NULL || !this->ax2550_comms_->is_connected()) {
    RCLCPP_ERROR(this->get_logger(), "cmd_vel_callback: ax2550 not connected");
    return;
  }
  RCLCPP_DEBUG(
    this->get_logger(), "cmd_vel_callback: cmd_vel.linear.x: %f, cmd_vel.angular.z: %f", cmd_vel.linear.x,
    cmd_vel.angular.z);
  // Convert mps to rpm
  double A = cmd_vel.linear.x;
  double B = cmd_vel.angular.z;

  double A_rpm = A / (M_PI * wheel_diameter_) * 60;
  double B_rpm = B / (M_PI * wheel_diameter_) * 60;

  // Convert rpm to relative
  double A_rel = A_rpm * speed_constant_;
  double B_rel = B_rpm * speed_constant_;

  // Bounds check
  if (A_rel > (double)this->max_a_) {
    RCLCPP_DEBUG(
      this->get_logger(), "cmd_vel_callback: A_rel: %f, this->max_a_: %f", A_rel, (double)this->max_a_);
    A_rel = (double)this->max_a_;
  }
  if (A_rel < -1 * (double)this->max_a_) {
    A_rel = -1 * (double)this->max_a_;
  }
  if (B_rel > (double)this->max_b_) {
    RCLCPP_DEBUG(
      this->get_logger(), "cmd_vel_callback: B_rel: %f, this->max_b_: %f", A_rel, (double)this->max_b_);
    B_rel = (double)this->max_b_;
  }
  if (B_rel < -1 * (double)this->max_b_) {
    B_rel = -1 * (double)this->max_b_;
  }
  RCLCPP_INFO(
    this->get_logger(), "cmd_vel_callback: A: %f, Arel: %f, B: %f, Brel: %f", A, A_rel, B,
    B_rel);

  this->ax2550_comms_->move(A_rel, B_rel);

}

void
Ax2550DriverNode::cmd_ax2550_callback(const std_msgs::msg::String & cmd_ax2550)
{
  RCLCPP_INFO(this->get_logger(), "Received cmd_ax2250 %s", cmd_ax2550.data.c_str());
  if (cmd_ax2550.data == "reset_encoders") {
    // reset encoders
    RCLCPP_INFO(this->get_logger(), "Resetting encoders");
    this->ax2550_comms_->reset_encoders();
    // reset odometry
    odometry.reset(
      new Odometry(
        wheel_diameter_, wheel_separation_, encoder_resolution_,
        this->now()));
  }
}

void
Ax2550DriverNode::publish_odometry_()
{
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped transform;
  this->odometry->get_messages(odom, transform, this->now(), odom_frame_id_, base_frame_id_);
  pub_odom_->publish(odom);
  if (this->enable_odom_tf_)
    tf_broadcaster_->sendTransform(transform);
}

void
Ax2550DriverNode::get_params_()
{
  try {
    this->device_name_ =
      this->declare_parameter<std::string>(std::string("device_name"), std::string("/dev/ttyUSB0"));
    RCLCPP_INFO(this->get_logger(), "device_name set to: %s", this->device_name_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The device_name provided was invalid");
    throw ex;
  }
  try {
    this->encoder_resolution_ =
      this->declare_parameter<double>(std::string("encoder_resolution"), 300.00);
    RCLCPP_INFO(this->get_logger(), "encoder_resolution set to: %f", this->encoder_resolution_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The encoder_resolution provided was invalid");
    throw ex;
  }
  try {
    this->encoder_poll_rate_ =
      this->declare_parameter<int>(std::string("encoder_poll_rate"), 100);
    RCLCPP_INFO(this->get_logger(), "encoder_poll_rate set to: %d", this->encoder_poll_rate_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The encoder_poll_rate provided was invalid");
    throw ex;
  }
  try {
    this->speed_constant_ =
      this->declare_parameter<double>(std::string("speed_constant"), 0.024);
    RCLCPP_INFO(this->get_logger(), "speed_constant set to: %f", this->speed_constant_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The speed_constant provided was invalid");
    throw ex;
  }
  try {
    this->amps_limit_ =
      this->declare_parameter<std::string>(std::string("amps_limit"), "");
    RCLCPP_INFO(this->get_logger(), "amps_limit set to: %s", this->amps_limit_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The amps_limt provided was invalid");
    throw ex;
  }
  try {
    this->max_a_ =
      this->declare_parameter<int>(std::string("max_a"), 127);
    RCLCPP_INFO(this->get_logger(), "max_a set to: %d", this->max_a_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The max_a provided was invalid");
    throw ex;
  }
  try {
    this->max_b_ =
      this->declare_parameter<int>(std::string("max_b"), 127);
    RCLCPP_INFO(this->get_logger(), "max_b set to: %d", this->max_b_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The max_b provided was invalid");
    throw ex;
  }
  try {
    this->wheel_separation_ =
      this->declare_parameter<double>(std::string("wheel_separation"), 0.315);
    RCLCPP_INFO(this->get_logger(), "wheel_separation set to: %f", this->wheel_separation_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The wheel_separation provided was invalid");
    throw ex;
  }
  try {
    this->wheel_diameter_ = this->declare_parameter<double>(std::string("wheel_diameter"), 0.07);
    RCLCPP_INFO(this->get_logger(), "wheel_diameter set to: %f", this->wheel_diameter_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The wheel_diameter provided was invalid");
    throw ex;
  }
  // AX2550 motor commands
  try {
    this->cmd_dir_fw_ =
      this->declare_parameter<std::string>(std::string("cmd_dir_fw"), std::string("!A"));
    RCLCPP_INFO(this->get_logger(), "cmd_dir_fw set to: %s", this->cmd_dir_fw_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The cmd_dir_fw provided was invalid");
    throw ex;
  }
  try {
    this->cmd_dir_bw_ =
      this->declare_parameter<std::string>(std::string("cmd_dir_bw"), std::string("!a"));
    RCLCPP_INFO(this->get_logger(), "cmd_dir_bw set to: %s", this->cmd_dir_bw_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The cmd_dir_bw provided was invalid");
    throw ex;
  }
  try {
    this->cmd_dir_le_ =
      this->declare_parameter<std::string>(std::string("cmd_dir_le"), std::string("!B"));
    RCLCPP_INFO(this->get_logger(), "cmd_dir_le set to: %s", this->cmd_dir_le_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The cmd_dir_le provided was invalid");
    throw ex;
  }
  try {
    this->cmd_dir_ri_ =
      this->declare_parameter<std::string>(std::string("cmd_dir_ri"), std::string("!b"));
    RCLCPP_INFO(this->get_logger(), "cmd_dir_ri set to: %s", this->cmd_dir_ri_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The cmd_dir_ri provided was invalid");
    throw ex;
  }
  // fram ids
  try {
    this->base_frame_id_ =
      this->declare_parameter<std::string>(std::string("base_frame_id"), std::string("base_link"));
    RCLCPP_INFO(this->get_logger(), "base_frame_id set to: %s", this->base_frame_id_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The base_frame_id provided was invalid");
    throw ex;
  }
  try {
    this->odom_frame_id_ =
      this->declare_parameter<std::string>(std::string("odom_frame_id"), std::string("odom"));
    RCLCPP_INFO(this->get_logger(), "odom_frame_id set to: %s", this->odom_frame_id_.c_str());
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The odom_frame_id provided was invalid");
    throw ex;
  }
  try {
    this->enable_odom_tf_ =
      this->declare_parameter<bool>(std::string("enable_odom_tf"), true);
    RCLCPP_INFO(this->get_logger(), "enable_odom_tf set to: %s", this->enable_odom_tf_ ? "true" : "false");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The enable_odom_tf provided was invalid");
    throw ex;
  }
}

} // namespace ax2550_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ax2550_driver::Ax2550DriverNode)

