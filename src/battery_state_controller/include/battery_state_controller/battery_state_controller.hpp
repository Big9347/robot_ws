#ifndef BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_
#define BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "battery_state_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/battery_state.hpp"

namespace battery_state_controller
{

class BatteryStateController : public controller_interface::ControllerInterface
{
public:
    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
   
    BATTERY_STATE_CONTROLLER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


protected:
  std::string sensor_name_;
  std::string frame_id_;

  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace battery_state_controller

#endif  // BATTERY_STATE_CONTROLLER__BATTERY_STATE_CONTROLLER_HPP_
