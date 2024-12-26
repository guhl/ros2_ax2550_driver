#include <string>
#include "ax2550_driver/ax2550_comms.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ax2550_driver
{
Ax2550Comms::Ax2550Comms()
: device_name_(std::string("")),
  connected_(false), synced_(false), old_speed_(0.0), old_direction_(0.0),
  ax2550_{new Ax2550()},
  cmd_dir_fw_("!A"), cmd_dir_bw_("!a"), cmd_dir_le_("!B"), cmd_dir_ri_("!b")
{
}

Ax2550Comms::~Ax2550Comms()
{
  this->ax2550_->disconnect();
}

void Ax2550Comms::set_device_name(std::string device_name)
{
  device_name_ = device_name;
  this->ax2550_->set_device_name(device_name);
}

void Ax2550Comms::init()
{
  this->ax2550_->set_device_name(device_name_);
  try {
    this->ax2550_->connect();
    this->connected_ = this->ax2550_->is_connected();
  } catch (const std::exception & ex) {
    // sleep and try again
    RCLCPP_WARN(rclcpp::get_logger("Ax2550Comms::init"),
      "device_name_: %s, could not connect status %s, try again", device_name_.c_str(), ex.what());
    sleep(1);
    try {
      this->ax2550_->connect();
      this->connected_ = this->ax2550_->is_connected();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("Ax2550Comms::init"),
        "device_name_: %s, could not connect status %s", device_name_.c_str(), ex.what());
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("Ax2550Comms::init"),
    "device_name_: %s, is connected: %s", device_name_.c_str(), (this->connected_ == true) ? "true" : "false");
  if (this->connected_)
    this->sync_();
}

bool
Ax2550Comms::is_connected()
{
  try {
    this->connected_ = this->ax2550_->is_connected();
    return this->connected_;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::is_connected"),
      "Error getting connection status %s", ex.what());
    return false;
  }
}

inline std::string
string_format(const std::string & fmt, ...)
{
  int size = 100;
  std::string str;
  va_list ap;
  while (1) {
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size) {
      str.resize(n);
      return str;
    }
    if (n > -1) {
      size = n + 1;
    } else {
      size *= 2;
    }
  }
  return str;
}

void
Ax2550Comms::move(double speed, double direction)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("Ax2550Comms::move"),
    "called, speed: %f, direction: %f", speed, direction);
  if (!this->ax2550_->is_connected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::move"),
      "must be connected to move");
    return;
  }
  unsigned char speed_hex, direction_hex;
  if (speed != old_speed_) {
    std::string fail;
    std::string cmd;
    std::string result;
    // Create the speed command
    speed_hex = (unsigned char) (fabs(speed));
    cmd =
      (speed < 0) ? cmd_dir_bw_ + string_format("%.2X", speed_hex) :
      cmd_dir_fw_ + string_format("%.2X", speed_hex);
    RCLCPP_DEBUG(
      rclcpp::get_logger("Ax2550Comms::move"),
      "AX2550:move-speed speed=%f, cmd= %s", speed, cmd.c_str());
    this->ax2550_->ack_nak_filt_clear();
    if (!this->ax2550_->issue_command(cmd, fail)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Ax2550Comms::move"),
        "command %s failed. Why: %s", cmd.c_str(), fail.c_str());
    } else {
      if (!this->ax2550_->get_ack_result(result)) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("Ax2550Comms::move"),
          "did not receive ACK on command %s", cmd.c_str());
      }
    }
    old_speed_ = speed;
  }
  if (direction != old_direction_) {
    std::string fail;
    std::string cmd;
    std::string result;
    // Create the direction command
    direction_hex = (unsigned char) (fabs(direction));
    cmd =
      (direction < 0) ? cmd_dir_ri_ + string_format("%.2X", direction_hex) :
      cmd_dir_le_ + string_format("%.2X", direction_hex);
    // Issue the direction command
    RCLCPP_DEBUG(
      rclcpp::get_logger("Ax2550Comms::move"),
      "AX2550:move-direction direction=%f, cmd= %s", speed, cmd.c_str());
    this->ax2550_->ack_nak_filt_clear();
    if (!this->ax2550_->issue_command(cmd, fail)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Ax2550Comms::move"),
        "command %s failed. Why: %s", cmd.c_str(), fail.c_str());
    } else {
      if (!this->ax2550_->get_ack_result(result)) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("Ax2550Comms::move"),
          "did not receive ACK on command %s", cmd.c_str());
      }
      old_direction_ = direction;
    }
  }
}

void
Ax2550Comms::reset_encoders()
{
  if (!this->ax2550_->is_connected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::reset_encoders"),
      "must be connected to reset encoders");
    return;
  }
  std::string fail;
  std::string cmd("!Q2");
  this->ax2550_->ack_nak_filt_clear();
  if (!this->ax2550_->issue_command(cmd, fail)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550CommsSerial::timer_callback"),
      "Failed to issue command: %s, fail: %s", cmd.c_str(), fail.c_str());
  } else {
    std::string result;
    if (!this->ax2550_->get_ack_result(result)) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("Ax2550CommsSerial::timer_callback"),
        "Did not receive ACK on command: %s", cmd.c_str());
    }
  }
}

void
Ax2550Comms::query_encoders(long & encoder1, long & encoder2, bool relative)
{
  if (!this->ax2550_->is_connected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::queryEncoders"),
      "must be connected to query encoders");
    return;
  }
  std::string cmd1, cmd2, fail1, fail2, result1, result2;
  this->ax2550_->encoders_filt_clear();
  cmd1 = (relative) ? "?Q4" : "?Q0";
  cmd2 = (relative) ? "?Q5" : "?Q1";
  bool enc_ret1 = this->ax2550_->issue_command(cmd1, fail1);
  bool enc_ret2 = this->ax2550_->issue_command(cmd2, fail2);
  if (!enc_ret1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::queryEncoders"),
      "Failed to issue command: %s, fail: %s", cmd1.c_str(), fail1.c_str());
  } else {
    this->ax2550_->get_enc_result(result1, encoder1);
  }
  if (!enc_ret2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::queryEncoders"),
      "Failed to issue command: %s, fail: %s", cmd2.c_str(), fail2.c_str());
  } else {
    this->ax2550_->get_enc_result(result2, encoder2);
  }
}

bool
Ax2550Comms::set_amps_limit(std::string limt)
{
  if (!this->ax2550_->is_connected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::queryEncoders"),
      "must be connected to set the amps limt");
    return false;
  }
  std::string cmd = "^02 " + limt;
  std::string fail;
  if (!this->ax2550_->issue_command(cmd, fail)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::set_amps_limit"),
      "Failed to issue command: %s, fail: %s", cmd.c_str(), fail.c_str());
      return false;
  } else {
    if (!this->ax2550_->save_config(fail)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Ax2550Comms::set_amps_limit"),
        "Failed to save the config. fail: %s ", fail.c_str());
        return false;
    } else {
      return true;
    }
  }
}

void
Ax2550Comms::sync_()
{
  if (this->synced_) {
    return;
  }
  /* in the original implementation the motion mode is set to 
  *  mixed mode by command "^01 C5\r"
  * this is not necessary and should be done externaly
  */
  if (!this->is_connected()){
      RCLCPP_ERROR(
        rclcpp::get_logger("Ax2550Comms::sync_"),
        "must be connected to sync controller");
      this->synced_ = false;
    return;
  }

  std::string fail("");
  this->ax2550_->wake_serial(fail);
  if (this->ax2550_->reset(fail)) {
    this->synced_ = true;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550Comms::sync_"),
      "Failed to reset the controller, fail: %s", fail.c_str());
    this->synced_ = false;
  }
}

} // namespace ax2550
