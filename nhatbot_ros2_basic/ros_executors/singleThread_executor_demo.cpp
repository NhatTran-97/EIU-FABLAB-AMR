#include "rclcpp/rclcpp.hpp"

/*However, this use case of Execute Callback is not useful. You do not even need the spin. So, 
create a new example with a Callback to a subscription topic "/box_bot_1/odom":*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("executor_example_1_node");
    RCLCPP_INFO(node->get_logger(), " I am duy nhat 1997");

// start and spin executor SingleThreaded
    //rclcpp::spin(node);

    // same code but in steps
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}