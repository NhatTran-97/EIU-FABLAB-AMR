#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("log_demo");
  rclcpp::WallRate loop_rate(0.5);
  //rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

        

  while (rclcpp::ok()) {

    RCLCPP_DEBUG(node->get_logger(), "There is a missing droid");
    RCLCPP_INFO(node->get_logger(), "The Emperor's cappuccino is done");
    RCLCPP_WARN(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
    RCLCPP_ERROR(node->get_logger(), "The rebels are breaking our defenses");
    RCLCPP_FATAL(node->get_logger(), "The DeathStar Is EXPLODING");

        
    rclcpp::spin_some(node);
    loop_rate.sleep();
        
  }
  rclcpp::shutdown();
  return 0;
}