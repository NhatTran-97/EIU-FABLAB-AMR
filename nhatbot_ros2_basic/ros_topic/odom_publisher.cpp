#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<rclcpp::Node>("odom_publisher", node_options);

    auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);

    rclcpp::Rate loop_rate(1); // 1 Hz publishing rate

    while (rclcpp::ok()) {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        // Fill in the necessary fields of the odom_msg here
        odom_msg->pose.pose.position.x = 1.0;
        odom_msg->pose.pose.position.y = 2.0;
        odom_msg->pose.pose.position.z = 0.0;

        // Publish the message
        odom_publisher->publish(*odom_msg);

        // Spin the node
        rclcpp::spin_some(node);

        // Sleep to achieve the desired publishing rate
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
