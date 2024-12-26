#ifndef AX2550_DRIVER__AX2550_COMMS_HPP_
#define AX2550_DRIVER__AX2550_COMMS_HPP_

#include "ax2550.hpp"

namespace ax2550_driver
{

class Ax2550Comms
{
public:
  Ax2550Comms();
  virtual ~Ax2550Comms();
  void set_device_name(std::string device_name);
  void set_commands(
    std::string cmd_dir_fw, std::string cmd_dir_bw, std::string cmd_dir_le,
    std::string cmd_dir_ri)
  {
    cmd_dir_fw_ = cmd_dir_fw;
    cmd_dir_bw_ = cmd_dir_bw;
    cmd_dir_le_ = cmd_dir_le;
    cmd_dir_ri_ = cmd_dir_ri;
  }
  void init();
  void move(double speed, double direction);
  void reset_encoders();
  void query_encoders(long & encoder1, long & encoder2, bool relative = false);
  bool set_amps_limit(std::string limt);
  bool is_connected();

private:
  void sync_();
  std::string device_name_;
  bool connected_;
  bool synced_;
  double old_speed_;
  double old_direction_;
  std::unique_ptr<Ax2550> ax2550_{};
  std::string cmd_dir_fw_;
  std::string cmd_dir_bw_;
  std::string cmd_dir_le_;
  std::string cmd_dir_ri_;
};

} // namespace ax2550_driver

#endif //AX2550_DRIVER__AX2550_COMMS_HPP_
