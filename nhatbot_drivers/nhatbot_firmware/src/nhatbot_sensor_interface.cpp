#include "nhatbot_firmware/nhatbot_sensor_interface.hpp"

namespace nhatbot_interface {

CallbackReturn NhatbotSensorInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    battery_voltage_ = 0.0;
    driver_temp_     = 0.0;
    motor_temp_      = 0.0;
    fault_left_      = 0.0;
    fault_right_     = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("NhatbotSensorInterface"), "✅ Sensor interface initialized.");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NhatbotSensorInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // dùng sensor[0] name làm prefix (ví dụ: "nhatbot_sensors")
    const std::string &sensor_name = info_.sensors[0].name;

    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "battery_voltage", &battery_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "driver_temp", &driver_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "motor_temp", &motor_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "fault_left", &fault_left_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "fault_right", &fault_right_));

    return state_interfaces;
}

hardware_interface::return_type NhatbotSensorInterface::read(const rclcpp::Time &, const rclcpp::Duration &)

{
    auto &driver = DriverManager::instance().driver();

    try {
        battery_voltage_ = driver.get_battery_voltage();
        driver_temp_     = driver.get_driver_temperature();
        motor_temp_      = driver.get_motor_temperature();

        auto [fl, fr] = driver.get_motor_faults();
        fault_left_  = static_cast<double>(fl);
        fault_right_ = static_cast<double>(fr);

        return hardware_interface::return_type::OK;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("NhatbotSensorInterface"),
                            "❌ Exception in read(): " << e.what());
        return hardware_interface::return_type::ERROR;
    }
}

} // namespace nhatbot_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_interface::NhatbotSensorInterface, hardware_interface::SensorInterface)
