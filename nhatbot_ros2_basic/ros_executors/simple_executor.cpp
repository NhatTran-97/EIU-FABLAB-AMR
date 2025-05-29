#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = 
                rclcpp::Node::make_shared("executor_01_node");
    
    RCLCPP_INFO(node->get_logger(),"hello world, I'm Nhat");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;

}