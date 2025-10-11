#include "nhatbot_firmware/sensor_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nhatbot_interface
{

// controller_interface::CallbackReturn SensorBroadcaster::on_init()
// {
//   auto ret = ControllerInterface::on_init();
//   if (ret != controller_interface::CallbackReturn::SUCCESS) 
//   {
//     return ret;
//   }
//   try 
//   {
//     battery_pub_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 10);
//     temp_pub_    = get_node()->create_publisher<std_msgs::msg::Float32>("~/motor_temp", 10);
//     reset_encoder_srv_ = get_node()->create_service<std_srvs::srv::Trigger>("reset_encoder", 
//     std::bind(&SensorBroadcaster::resetEncoderCb, this, std::placeholders::_1, std::placeholders::_2));
//     RCLCPP_INFO(get_node()->get_logger(),
//                 "✅ SensorBroadcaster initialized. Publishing battery_state + motor_temp, service ~/reset_encoder ready.");
//   } 
//   catch (const std::exception &e) 
//   {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed init SensorBroadcaster: %s", e.what());
//     return controller_interface::CallbackReturn::ERROR;
//   }
//   return controller_interface::CallbackReturn::SUCCESS;
// }


controller_interface::CallbackReturn SensorBroadcaster::on_init()
{
  try
  {
    battery_pub_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 10);
    temp_pub_    = get_node()->create_publisher<std_msgs::msg::Float32>("~/motor_temp", 10);

    reset_encoder_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/reset_encoder",
        std::bind(&SensorBroadcaster::resetEncoderCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_node()->get_logger(), "✅ SensorBroadcaster initialized with publishers + service.");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed init SensorBroadcaster: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}




controller_interface::InterfaceConfiguration SensorBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration SensorBroadcaster::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        "nhatbot_sensors/battery_voltage",
        "nhatbot_sensors/motor_temp"
      }};
}

controller_interface::return_type SensorBroadcaster::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  double bat  = state_interfaces_[0].get_value();
  double temp = state_interfaces_[1].get_value();

  sensor_msgs::msg::BatteryState bat_msg;
  bat_msg.voltage = bat;
  battery_pub_->publish(bat_msg);

  std_msgs::msg::Float32 temp_msg;
  temp_msg.data = temp;
  temp_pub_->publish(temp_msg);

  return controller_interface::return_type::OK;
}

// void SensorBroadcaster::resetEncoderCb(
//     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
// {
//   auto &driver = DriverManager::instance().driver();


//   bool ok = DriverManager::instance().clearPosition(zlac_modbus::ClearPos::BOTH);

//   if (driver.clear_position(zlac_modbus::ClearPos::BOTH)) {
//     response->success = true;
//     response->message = "Encoders reset successfully (both sides)";
//   } else {
//     response->success = false;
//     response->message = "Failed to reset encoders";
//   }
// }

void SensorBroadcaster::resetEncoderCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto &mgr = DriverManager::instance();
    if (mgr.clearPosition(zlac_modbus::ClearPos::BOTH)) {
        response->success = true;
        response->message = "Encoders reset successfully (both sides)";
    } else {
        response->success = false;
        response->message = "Failed to reset encoders";
    }
}





}  // namespace nhatbot_interface

PLUGINLIB_EXPORT_CLASS(nhatbot_interface::SensorBroadcaster, controller_interface::ControllerInterface)
