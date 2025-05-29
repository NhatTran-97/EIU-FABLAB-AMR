#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomSubscriber : public rclcpp::Node
{
    public:
        OdomSubscriber(std::string odom_topic_name) : Node("simple_subscriber")
        {
            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_name, 10,
                std::bind(&OdomSubscriber::topic_callback, this, std::placeholders::_1));
        }
    private:
        void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Odometry=['%f', '%f','%f']",
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    std::shared_ptr<OdomSubscriber> node = std::make_shared<OdomSubscriber>("/odom");
    RCLCPP_INFO(node->get_logger(), "I am Duy Nhat 1997");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}