#ifndef BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_
#define BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/float64.hpp"

namespace battery_state_controller
{

class BatteryStateController : public controller_interface::ControllerInterface
{
public:
    BatteryStateController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    controller_interface::return_type update() override;

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  std::vector<std::string> sensor_names_;
  rclcpp::Time last_publish_time_;
  double publish_rate_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> battery_state_publisher_;
  std_msgs::msg::Float64 battery_state_msg_;
};
}  // namespace battery_state_controller
#endif  // BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_
