#include "battery_state_controller/battery_state_controller.hpp"

namespace battery_state_controller
{

BatteryStateController::BatteryStateController()
: publish_rate_(1.0)
{
}

controller_interface::return_type BatteryStateController::init(const std::string & controller_name)
{
  // Create a ROS 2 node.
  node_ = rclcpp::Node::make_shared(controller_name + "_node");

  // Get the publish rate from the controller parameters.
  node_->get_parameter_or<double>("publish_rate", publish_rate_, 1.0);

  // Create a battery state publisher.
  battery_publisher_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

  // Retrieve the battery state handle from the hardware interface.
  battery_handle_ = hardware_interface::BatteryStateHandle("battery", "voltage", "current", "charge", "capacity");
  battery_handle_ptr_ = std::make_shared<hardware_interface::Battery
