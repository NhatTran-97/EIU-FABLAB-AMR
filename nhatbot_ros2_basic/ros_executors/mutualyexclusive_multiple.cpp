
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node
{
    public:
        TwoTimer(float sleep_timer1, float sleep_timer2) : Node("slow_timer_subscriber")
        {
            // In this example, create two callback groups:
            callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            this->wait_time1 = sleep_timer1;
            this->wait_time2 = sleep_timer2;

            timer1_ = this->create_wall_timer(
                500ms, std::bind(&TwoTimer::timer1_callback, this), callback_group_1);
            timer2_ = this->create_wall_timer(
                500ms, std::bind(&TwoTimer::timer2_callback, this), callback_group_2);
        }
    private:
        void timer1_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
            sleep(this->wait_time1);
            RCLCPP_INFO(this->get_logger(), "Timer 1 Callback END");
        }
        void timer2_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer 2 Callback Start");
            sleep(this->wait_time2);
            RCLCPP_INFO(this->get_logger(), "Timer 2 Callback END");
        }
    rclcpp::CallbackGroup::SharedPtr callback_group_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_2;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    float wait_time1;
    float wait_time2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // Instantiate the Node
  float sleep_time1 = 1.0;
  float sleep_time2 = 3.0;

  std::shared_ptr<TwoTimer> two_timer_node = std::make_shared<TwoTimer>(sleep_time1, sleep_time2);
   // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}