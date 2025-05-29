#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class movingRobot : public rclcpp::Node
{
    private:
        geometry_msgs::msg::Twist twist;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        movingRobot() : Node("Moving_robot_node")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer_ = this->create_wall_timer(1000ms, std::bind(&movingRobot::timer_callback, this));
            
        }

        void timer_callback()
        {
            this->timer_->cancel(); // cancel timer
            this->main_logic();
        }

        void turn()
        {
            RCLCPP_INFO(this->get_logger(), "Turning...");
            twist.linear.x = 0.5;
            twist.angular.z = 0.5;
            publisher_-> publish(twist);
        }
        void main_logic()
        {
            turn();
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<movingRobot>());
    rclcpp::shutdown();
    return 0;
}