#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // Initialize node name
    auto node = rclcpp::Node::make_shared("simple_publisher");

    //Create a Publisher object that will publish on the /counter topic, message of the type Int32
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter",10);

    // create a variable named "message" of type Int32
    auto message = std::make_shared<std_msgs::msg::Int32>();
   
    // Initialize "message" Variable
    message->data = 0;

    // Set a publish rate of 2 Hz
    rclcpp::WallRate loop_rate(2);

    while (rclcpp::ok())
    {
        publisher -> publish(*message);
        message->data++;
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}