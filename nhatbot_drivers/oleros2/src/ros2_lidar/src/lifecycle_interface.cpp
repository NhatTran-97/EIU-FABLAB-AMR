#include <string>

#include "ros2_lidar/interfaces/lifecycle_interface.hpp"

namespace lifecycle_interface
{

LifecycleInterface::LifecycleInterface(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(name, options), is_active(false)
{
}

CallbackReturn LifecycleInterface::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring lidar driver node.");
  onConfigure();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Activating lidar driver node.");
  onActivate();
  is_active = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating lidar driver node.");
  onDeactivate();
  is_active = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(this->get_logger(), "Handing error in lidar driver node.");
  onError();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down lidar driver node.");
  onShutdown();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleInterface::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up lidar driver node.");
  onCleanup();
  return CallbackReturn::SUCCESS;
}

}  // namespace lifecycle_interface
