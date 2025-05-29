#include "nhatbot_ros2_basic/moverobot_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace my_components
{
    MoveRobot::MoveRobot(const rclcpp::NodeOptions & options)
    : Node("moverobot", options)
    {
    
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(1s, std::bind(&MoveRobot::on_timer, this));
    }

    void MoveRobot::on_timer()
    {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.3;
    msg->angular.z = 0.3;
    std::flush(std::cout);

    pub_->publish(std::move(msg));
    }

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::MoveRobot)