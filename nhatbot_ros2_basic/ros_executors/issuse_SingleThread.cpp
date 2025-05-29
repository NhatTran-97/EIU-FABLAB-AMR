#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <chrono>
using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node
{
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        
        
    
    public:
        OdomSubscriber(std::string odom_topic_name) : Node("odom_subscriber")
        {
            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_name,10,
                std::bind(&OdomSubscriber::topic_callback, this, std::placeholders::_1));
        }
    
    private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odometry=['%f', '%f', '%f']",
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    }
    

};

class SlowTimer : public rclcpp::Node
{
    public:
        SlowTimer(float sleep_timer) : Node("sub_timer")
        {
            this->wait_time = sleep_timer;
            timer_ = this->create_wall_timer(
                500ms, std::bind(&SlowTimer::timer_callback, this));
        }

    private:
    void timer_callback()
    {
        sleep(this->wait_time);
        RCLCPP_INFO(this->get_logger(), "TICK");
    }
        float wait_time;
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OdomSubscriber> odom_subs_node = std::make_shared<OdomSubscriber>("/robot/odom");
    float sleep_timer = 3.0;
    std::shared_ptr<SlowTimer> slow_timer_node = std::make_shared<SlowTimer>(sleep_timer);

    RCLCPP_INFO(odom_subs_node->get_logger(), "odom_subs_node INFO...");
    RCLCPP_INFO(slow_timer_node->get_logger(), "slow_timer_node INFO...");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(odom_subs_node);
    executor.add_node(slow_timer_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;

    return 0;
}