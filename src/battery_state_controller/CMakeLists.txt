cmake_minimum_required(VERSION 3.5)
project(battery_state_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(
  battery_state_controller
  SHARED
  src/battery_state_controller.cpp
)
target_include_directories(
  battery_state_controller
  PUBLIC
  include
)
ament_target_dependencies(
  battery_state_controller
  sensor_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(battery_state_controller PRIVATE "BATTERY_STATE_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(battery_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface battery_state_controller.xml)

install(
  TARGETS
  battery_state_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_battery_state_controller
    test/test_load_battery_state_controller.cpp
  )
  target_include_directories(test_load_battery_state_controller PRIVATE include)
  ament_target_dependencies(test_load_battery_state_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
  # prevent pluginlib from using boost
  target_compile_definitions(test_load_battery_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_battery_state_controller
    test/test_battery_state_controller.cpp
  )
  target_include_directories(test_battery_state_controller PRIVATE include)
  target_link_libraries(test_battery_state_controller
    battery_state_controller
  )
  ament_target_dependencies(test_battery_state_controller
    hardware_interface
  )
  ament_target_dependencies(test_load_battery_state_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  battery_state_controller
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  hardware_interface
)

ament_package()