
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ax2550_driver/ax2550_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ax2550_driver::Ax2550DriverNode>(rclcpp::NodeOptions());

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
