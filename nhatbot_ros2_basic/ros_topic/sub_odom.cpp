#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node
{
    private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

    public:
    OdomSubscriber():Node("sub_odom")
    {
        odomSub_ = this -> create_subscription<nav_msgs::msg::Odometry>(
            "odom",10, std::bind(&OdomSubscriber::Odom_callback, this,_1));
    }

    void Odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odom sub: '%f'",msg->pose.pose.position.x);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}