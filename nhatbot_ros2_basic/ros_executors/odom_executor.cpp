#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomSubscriber : public rclcpp::Node {
    public:
        OdomSubscriber(std::string odom_topic_name) : Node("subOdom_Executor_Node")
        {
            subOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_name, 10,
                std::bind(&OdomSubscriber::topic_callback, this, std::placeholders::_1));

        }
    private:
        void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Odometry = ['%f','%f','%f']",
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OdomSubscriber> node = std::make_shared<OdomSubscriber>("/robot/odom");

    RCLCPP_INFO(node->get_logger(), "odom infomation");
    rclcpp::executors::SingleThreadedExecutor executor_odom;
    executor_odom.add_node(node);
    executor_odom.spin();

    rclcpp::shutdown();
    return 0;
}