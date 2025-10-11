#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nhatbot_firmware/driver_manager.hpp" 


namespace nhatbot_interface {

class SensorBroadcaster : public controller_interface::ControllerInterface {
public:
  SensorBroadcaster() = default;

  // Bắt buộc trong ROS2 Humble
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;

    // new: service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoder_srv_;

void resetEncoderCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};
} // namespace nhatbot_interface