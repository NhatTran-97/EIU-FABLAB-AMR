#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberQos : public rclcpp::Node
{
    public:
    SubscriberQos() : Node("subscriber_qos_obj")
    {
        rclcpp::QoS qos_profile(10);
        /* 
        reliability=ReliabilityPolicy.RELIABLE
        This means you are forcing this subscriber to receive every message sent by the publisher.
        */
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "qos_test", qos_profile, std::bind(&SubscriberQos::listener_callback, this, _1));
            
    }
    private:
        void listener_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Data Received = '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberQos>());
    rclcpp::shutdown();
    return 0;
}