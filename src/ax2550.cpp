#include "ax2550_driver/ax2550.hpp"
#include "rclcpp/rclcpp.hpp"
#include <climits>

namespace ax2550_driver
{

Ax2550::Ax2550()
{
}

Ax2550::~Ax2550()
{
  disconnect();
}

void Ax2550::set_device_name(std::string device_name)
{
  this->device_name_ = device_name;
}

void Ax2550::connect()
{
  if (this->connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("Ax2550::connect"), "Serial port already connected");
    return;
  }
  // Check to see if the port is set to something
  if (this->device_name_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("Ax2550::connect"), "Port not set to a device name");
    return;
  }
  // Call disconnect to ensure we are in a clean state
  this->disconnect();
  // Setup the filters
  this->setup_filters_();
  // Setup the serial port
  try {
    this->serial_port_ = new serial::Serial();
    this->serial_port_->setPort(this->device_name_);
    this->serial_port_->setBaudrate(9600);
    this->serial_port_->setParity(serial::parity_even);
    this->serial_port_->setStopbits(serial::stopbits_one);
    this->serial_port_->setBytesize(serial::sevenbits);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    this->serial_port_->setTimeout(to);
    // Open the serial port
    this->serial_port_->open();
    // Setup the serial listener
    this->serial_listener_.setChunkSize(2);
    this->serial_listener_.startListening((*this->serial_port_));
    this->connected_ = true;
  } catch (serial::IOException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Ax2550::connect"), 
      "Exception while connecting: %s", ex.what());
  }
}

void Ax2550::disconnect()
{
  if (is_connected()) {
    this->connected_ = false;
    if (this->serial_listener_.isListening()) {
      this->serial_listener_.stopListening();
    }
    if (this->serial_port_ != NULL) {
      delete this->serial_port_;
      this->serial_port_ = NULL;
    }
  }
}

bool Ax2550::is_connected()
{
  return this->connected_;
}

bool
Ax2550::issue_command(const std::string & command, std::string & fail_why)
{
  if (this->is_connected()) {
    // Setup an echo filter
    serial::utils::BufferedFilterPtr echo_filt =
      this->serial_listener_.createBufferedFilter(serial::utils::SerialListener::exactly(command));
    this->serial_port_->write(command + "\r");
    // Listen for the echo
    if (echo_filt->wait(50).empty()) {
      fail_why = "failed to receive an echo";
      return false;
    }
    return true;
  } else {
      fail_why = "serial port is not connected";
      return false;
  }
}

bool
Ax2550::get_ack_result(std::string & result, bool clear)
{
  bool ret = false;
  if (clear) {
    this->ack_nak_filt_->clear();
  }
  std::string res = this->ack_nak_filt_->wait(100);
  if (res != "+") {
    if (res == "-") {
      RCLCPP_DEBUG(rclcpp::get_logger("Ax2550::get_ack_result"), "Received NAK");
    } else {
      RCLCPP_DEBUG(
        rclcpp::get_logger("Ax2550::get_ack_result"),
        "Did not receive ACK or NAK");
    }
  } else {
    result = res;
    ret = true;
    RCLCPP_DEBUG(rclcpp::get_logger("Ax2550::get_ack_result"), "Received: '%s'", res.c_str());
  }
  return ret;
}

bool
Ax2550::reset(std::string &fail_why)
{
  if (this->is_connected()) {
    // Setup an echo filter
    serial::utils::BufferedFilterPtr ok_filt =
      this->serial_listener_.createBufferedFilter(serial::utils::SerialListener::contains("OK"));
    this->serial_port_->write("%rrrrrr\r");
    // Listen for the echo
    if (ok_filt->wait(2000).empty()) {
      fail_why = "failed to receive OK";
      return false;
    }
    return true;
  } else {
      fail_why = "serial port is not connected";
      return false;
  }
}

bool
Ax2550::save_config(std::string &fail_why)
{
  if (this->is_connected()) {
    // Setup an echo filter
    serial::utils::BufferedFilterPtr ok_filt =
      this->serial_listener_.createBufferedFilter(serial::utils::SerialListener::contains("OK"));
    this->serial_port_->write("^FF\r");
    // Listen for the echo
    if (ok_filt->wait(2000).empty()) {
      fail_why = "failed to receive OK";
      return false;
    }
    return true;
  } else {
      fail_why = "serial port is not connected";
      return false;
  }
}

bool
Ax2550::wake_serial(std::string &fail_why)
{
  if (this->is_connected()) {
    // Setup an echo filter
    serial::utils::BufferedFilterPtr ok_filt =
      this->serial_listener_.createBufferedFilter(serial::utils::SerialListener::contains("OK"));
    this->serial_port_->write("\r\r\r\r\r\r\r\r\r\r");
    // Listen for the echo
    if (ok_filt->wait(2000).empty()) {
      fail_why = "failed to receive OK";
      return false;
    }
    return true;
  } else {
      fail_why = "serial port is not connected";
      return false;
  }
}

void
Ax2550::ack_nak_filt_clear()
{
  this->ack_nak_filt_->clear();
}

void
Ax2550::encoders_filt_clear()
{
  this->encoders_filt_->clear();
}

bool
Ax2550::get_enc_result(std::string & result, long & enc_value, bool clear)
{
  bool ret = false;
  if (clear) {
    this->encoders_filt_->clear();
  }
  std::string res = this->encoders_filt_->wait(100);
  if (res.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("Ax2550::get_enc_result"), "Failed to get a result");
  } else {
    result = res;
    enc_value = convert_encoder_value_(res);
    ret = true;
    RCLCPP_DEBUG(
      rclcpp::get_logger("Ax2550::get_enc_result"),
      "Received: '%s', which translates to '%lu'", res.c_str(), enc_value);
  }
  return ret;
}

long
Ax2550::convert_encoder_value_(std::string value)
{
  int32_t encVal;
  int i;
  std::string firstChar = value.substr(0, 1);
  bool iscon = from_string<int>(i, firstChar, std::hex);
  if (!iscon) {
    return LONG_MAX;
  }
  std::string fillStr;
  if (i >= 8) {
    fillStr = std::string("FFFFFFFF");
  } else {
    fillStr = std::string("00000000");
  }
  std::string retVar = fillStr.substr(value.size(), fillStr.size()) + value;
  retVar = std::string("0x") + retVar;
  encVal = strtoul(retVar.c_str(), NULL, 16);
  return encVal;
}

inline bool
isAnEncoderMsg(const std::string & token)
{
  std::string test = "0123456789abcdefABCDEF";
  // If token[0] is any of 0123456789abcdefABCDEF (hex)
  if (token.substr(0, 1).find_first_of(test) != std::string::npos) {
    return true;
  }
  return false;
}

inline bool
isAckOrNak(const std::string & token)
{
  std::string test = "+-";
  if (token.find_first_of(test) != std::string::npos) {
    return true;
  }
  return false;
}

void
Ax2550::setup_filters_()
{
  // Setup the encoder filter
  this->encoders_filt_ =
    this->serial_listener_.createBufferedFilter(isAnEncoderMsg);
  // Setup ack/nak filter
  this->ack_nak_filt_ =
    this->serial_listener_.createBufferedFilter(isAckOrNak);
}

} // namespace ax2550
