#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lidar/driver_types.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ros2_lidar::OS1Driver>(options);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
