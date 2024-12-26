cmake_minimum_required(VERSION 3.8)
project(ax2550_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)

include_directories(
  include
)

set(HEADER_FILES 
  include/serial/serial.h 
  include/serial/v8stdint.h 
  include/serial/utils/serial_listener.h 
  include/serial/utils/concurrent_queue.h
  include/ax2550_driver/ax2550.hpp)

set(AX2550 ax2550)

add_library(${AX2550} SHARED
  src/ax2550.cpp
  src/serial/serial.cc 
  src/serial/serial_listener.cpp 
  src/serial/impl/unix.cc
  src/serial/impl/win.cc 
  src/serial/impl/list_ports/list_ports_linux.cc 
  src/serial/impl/list_ports/list_ports_osx.cc 
  src/serial/impl/list_ports/list_ports_win.cc 
  ${HEADER_FILES}
)
# create ament index resource which references the libraries in the binary dir
set(node_plugins "")
ament_target_dependencies(ax2550 rclcpp)

target_link_libraries(${AX2550} ${Boost_LIBRARIES})

install(
  TARGETS ${AX2550}
  EXPORT ${AX2550}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

set(AX2550_COMMS ax2550_comms)
add_library(${AX2550_COMMS} SHARED src/ax2550_comms.cpp)
target_link_libraries(${AX2550_COMMS} ax2550)

install(
  TARGETS ${AX2550_COMMS}
  EXPORT ${AX2550_COMMS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${AX2550})
ament_export_libraries(${AX2550_COMMS})

add_library(ax2550_driver_node SHARED src/ax2550_driver_node.cpp src/ax2550_odometry.cpp)
target_link_libraries(ax2550_driver_node ax2550_comms)
target_compile_definitions(ax2550_driver_node PRIVATE "SSC32U_BUILDING_DLL")
ament_target_dependencies(ax2550_driver_node rclcpp rclcpp_components geometry_msgs nav_msgs std_msgs tf2_geometry_msgs tf2_ros)
rclcpp_components_register_nodes(ax2550_driver_node "ax2550_driver::Ax2550DriverNode")
set(node_plugins "${node_plugins}ax2550_driver::Ax2550DriverNode;$<TARGET_FILE:ax2550_driver_node>\n")

add_executable(ax2550_driver
  src/ax2550_driver.cpp
)

target_link_libraries(ax2550_driver ax2550_comms ax2550_driver_node)

install(TARGETS
  ax2550_driver_node
  ax2550_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
