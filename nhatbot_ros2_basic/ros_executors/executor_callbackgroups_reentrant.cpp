#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

// For the parameters, templates of seconds or ms

/*
• You can see that it works perfectly. Because Callback 1 is called every one second and Callback 2 every three seconds. 
  You see that exact behavior, so they work in parallel.
*/
using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node {
public:
  TwoTimer(float sleep_timer1, float sleep_timer2) : Node("slow_timer_subscriber") 
      
    {

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        

    this->wait_time1 = sleep_timer1; this->wait_time2 = sleep_timer2;
    

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_callback_1, this), callback_group_);

    timer2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_callback_2, this), callback_group_);
  }

private:
  void timer_callback_1() 
  {
    RCLCPP_INFO(this->get_logger(), "Wating...TIMER CALLBACK 1");
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "End TIMER CALLBACK 1");
  }

  void timer_callback_2() 
  {
    RCLCPP_INFO(this->get_logger(), "Wating...TIMER CALLBACK 2");
    sleep(this->wait_time2);
    RCLCPP_INFO(this->get_logger(), "End TIMER CALLBACK 2");
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  float wait_time1;
  float wait_time2;
};

int main(int argc, char *argv[]) {
  // Some initialization.
  rclcpp::init(argc, argv);

  // Instantiate a Node.
  float sleep_time1 = 1.0;
  float sleep_time2 = 3.0;
  std::shared_ptr<TwoTimer> two_timer_node =
      std::make_shared<TwoTimer>(sleep_time1, sleep_time2);

  rclcpp::Node::make_shared("executor_example_5_node");

  // This is the same as a print in ROS
  RCLCPP_INFO(two_timer_node->get_logger(), "timer_1_node INFO...");

  // Same code, but in steps
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();

  // Shut down and exit.
  rclcpp::shutdown();
  return 0;
}