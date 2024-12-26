import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  ax2550_driver_node = launch_ros.actions.Node(
    package = "ax2550_driver",
    executable = "ax2550_driver",
    parameters = [os.path.join(get_package_share_directory('ax2550_driver'),'config','ax2550.config.yaml')],
    name = "ax2550_driver_node"
  )

  return launch.LaunchDescription([
    ax2550_driver_node
  ])
