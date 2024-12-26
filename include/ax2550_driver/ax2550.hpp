#ifndef AX2550_DRIVER__AX2550_HPP_
#define AX2550_DRIVER__AX2550_HPP_

#include <string>
#include "serial/serial.h"
#include "serial/utils/serial_listener.h"

namespace ax2550_driver
{

template<class T>
bool from_string(
  T & t,
  const std::string & s,
  std::ios_base & (*f)(std::ios_base &))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

class Ax2550
{
public:
  Ax2550();
  virtual ~Ax2550();
  void set_device_name(std::string device_name);
  void connect();
  void disconnect();
  bool is_connected();
  bool issue_command(const std::string & command, std::string & fail_why);
  bool reset(std::string & fail_why);
  bool save_config(std::string & fail_why);
  bool wake_serial(std::string & fail_why);
  void ack_nak_filt_clear();
  void encoders_filt_clear();
  bool get_ack_result(std::string & result, bool clear = false);
  bool get_enc_result(std::string & result, long & enc_value, bool clear = false);

private:
  long convert_encoder_value_(std::string value);

  std::string device_name_;
  serial::Serial * serial_port_;
  serial::utils::SerialListener serial_listener_;
  // Filter stuff
  void setup_filters_();
  serial::utils::BufferedFilterPtr encoders_filt_;
  serial::utils::BufferedFilterPtr ack_nak_filt_;
  bool connected_;
};

} // namespace ax2550

#endif // AX2550_DRIVER__AX2550_HPP_
