#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber(std::string odom_topic_name) : Node("odom_subscriber") {

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name, 10,
        std::bind(&OdomSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odometry=['%f','%f','%f']",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

class SlowTimer : public rclcpp::Node {
public:
  SlowTimer(float sleep_timer) : Node("slow_timer_subscriber") {
    this->wait_time = sleep_timer;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SlowTimer::timer_callback, this));
  }

private:
  void timer_callback() {
    sleep(this->wait_time);
    RCLCPP_INFO(this->get_logger(), "TICK");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the odometry subscriber node
  std::shared_ptr<OdomSubscriber> odom_subs_node =
      std::make_shared<OdomSubscriber>("/robot/odom");

  // Instantiate the timer node node
  float sleep_time = 3.0;
  std::shared_ptr<SlowTimer> slow_timer_node =
      std::make_shared<SlowTimer>(sleep_time);

  RCLCPP_INFO(odom_subs_node->get_logger(), "odom_subs_node INFO...");
  RCLCPP_INFO(slow_timer_node->get_logger(), "slow_timer_node INFO...");

  // Initialize two executor objects
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::executors::StaticSingleThreadedExecutor executor2;
    
  executor.add_node(odom_subs_node);
  executor2.add_node(slow_timer_node);
  executor.spin();
  executor2.spin();

  rclcpp::shutdown();
  return 0;
}