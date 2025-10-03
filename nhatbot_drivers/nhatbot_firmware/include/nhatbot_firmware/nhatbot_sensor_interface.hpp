#ifndef NHATBOT_SENSOR_INTERFACE_HPP
#define NHATBOT_SENSOR_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nhatbot_firmware/driver_manager.hpp"
#include <vector>
#include <string>

namespace nhatbot_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NhatbotSensorInterface : public hardware_interface::SensorInterface
{
public:
    NhatbotSensorInterface() = default;
    ~NhatbotSensorInterface() override = default;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
    double battery_voltage_{0.0};
    double driver_temp_{0.0};
    double motor_temp_{0.0};
    double fault_left_{0.0};
    double fault_right_{0.0};
};

} // namespace nhatbot_interface

#endif
