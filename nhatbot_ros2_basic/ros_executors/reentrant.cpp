#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node
{
    public:
    TwoTimer(float sleep_timer1, float sleep_timer2) : Node("slow_timer_subscriber")
    {
        /*
        create a callback group with the line of code shown below
        */
        callback_group_  = this-> create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        this->wait_time1 = sleep_timer1;
        this->wait_time2 = sleep_timer2;
/*
The, pass callback_group_ as one additional argument to create_wall_timer(). In this case, both use the 
same callback group of type Reentrant .
Similarly, pass a callback group into subscribers, services or action
*/
        timer1_ = this-> create_wall_timer(500ms, std::bind(&TwoTimer::timer1_callback, this), callback_group_);
        timer2_ = this-> create_wall_timer(500ms, std::bind(&TwoTimer::timer2_callback, this), callback_group_);

    }
    private:
        void timer1_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer 1 callback start");
            sleep(this->wait_time1);
            RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");

        }
        void timer2_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Timer 2 callback start");
            sleep(this->wait_time2);
            RCLCPP_INFO(this->get_logger(), "Timer 2 Callback End");
            
        }

        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        float wait_time1;
        float wait_time2;
        
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    float sleep_time1 = 1.0;
    float sleep_time2 = 3.0;

    std::shared_ptr<TwoTimer> two_timer_node = std::make_shared<TwoTimer>(sleep_time1,sleep_time2);

    // Initialize one MultiThreadedExecutor object
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(two_timer_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}