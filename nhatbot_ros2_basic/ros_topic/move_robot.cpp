/*
    Inside the package, create a new C++ script named move_robot.cpp
    Inside a new script, create a Publisher node that publishes data to the /cmd_vel Topic to move robot in circles.
    Compile my own package
    Launch the program and check that the robot moves correctly
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chrono"

using namespace std::chrono_literals;


class Moverobot : public rclcpp::Node
{
 public:
    Moverobot():Node("move_robot")
    {
        publisher_ = this-> create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        timer_ = this-> create_wall_timer(
            500ms, std::bind(&Moverobot::timer_callback, this));   
    }
 private: 
 void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.2;
        message.angular.z = 0.2;
        publisher_ -> publish(message);   
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Moverobot>());
    rclcpp::shutdown();
    return 0;
}