#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node 
{
public:
  MoveRobot() : Node("test") 
  {

    this->declare_parameter("turning_speed", 0.0);
    this->declare_parameter("forward_speed", 0.0);

    mode = "turning";
    getting_params();

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&MoveRobot::timer_callback, this));
  }

  void getting_params() 
  {
    turning_speed =
        this->get_parameter("turning_speed").get_parameter_value().get<float>();
    forward_speed =
        this->get_parameter("forward_speed").get_parameter_value().get<float>();
  }

  void timer_callback() 
  {
    if (mode == "turning") 
    {
      go_forward();
      mode = "go_forward";
    } 
    else if (mode == "go_forward") 
    {
      turn();
      mode = "turning";
    } else 
    {
    }
  }

  void turn() 
  {
    twist.linear.x = 0.0;
    twist.angular.z = turning_speed;
    RCLCPP_INFO(this->get_logger(), "TURNING AT SPEED ==> %f", turning_speed);
    publisher_->publish(twist);
  }

  void go_forward() 
  {
    twist.linear.x = forward_speed;
    twist.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "GOING FORWARD AT SPEED ==> %f",
                forward_speed);
    publisher_->publish(twist);
  }

private:
  std::string mode;
  float turning_speed;
  float forward_speed;
  geometry_msgs::msg::Twist twist;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}