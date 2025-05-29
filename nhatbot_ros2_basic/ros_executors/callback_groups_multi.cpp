#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

/*
Here, you have a class Node with MORE than one Callback in it, in this case, TWO.
- 
- See what happens, because based on what you explained before, the **Multi-Threaded Executor** only launches 
**ONE THREAD per Node if you do not specify otherwise**.
The reason that only one Callback is executed is that the Executor only created one thread per Node. So you only have one thread to execute everything; 
therefore, you cannot execute several Callbacks in parallel.
*/

// For the parameters, templates of seconds or ms
using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node 
{
public:
  TwoTimer(float sleep_timer1, float sleep_timer2) : Node("slow_timer_subscriber")      
 {

    this->wait_time1 = sleep_timer1;
    this->wait_time2 = sleep_timer2;

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_callback_1, this));

    timer2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_callback_2, this));
  }

private:
  void timer_callback_1() 
  {
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "TIMER CALLBACK 1");
  }

  void timer_callback_2() 
  {
    sleep(this->wait_time2);
    RCLCPP_INFO(this->get_logger(), "TIMER CALLBACK 2");
  }

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

  rclcpp::Node::make_shared("executor_callbackGroup_node");

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