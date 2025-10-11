#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_lidar/driver_types.hpp"
#include "ros2_lidar/OS1/OS1_sensor.hpp"

namespace ros2_lidar
{
ros2_lidar::OS1Driver::OS1Driver(rclcpp::NodeOptions options)
: lidarDriver{std::make_unique<OS1::OS1Sensor>(), options} {}
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_lidar::OS1Driver)
