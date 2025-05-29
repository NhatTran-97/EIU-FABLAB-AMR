#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberQoS : public rclcpp::Node
{
    public:
        SubscriberQoS(int &argc [[maybe_unused]], char **argv) : Node("subscriber_qos_obj")
        {
            durability = argv[2];
            rclcpp::QoS qos_profile_subscriber(10);
            if(durability == "transient_local")
            {
                qos_profile_subscriber.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            }
            else
            {

            }
            rclcpp::SubscriptionOptions subscription_options;
            subscription_options.event_callbacks.incompatible_qos_callback = 
            std::bind(&SubscriberQoS::incompatible_qos_info_callback, this,_1);
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "qos_test", qos_profile_subscriber,
                std::bind(&SubscriberQoS::listener_callback, this,_1),
                subscription_options);
        }
        
    private:
        void incompatible_qos_info_callback(
            rclcpp::QOSRequestedIncompatibleQoSInfo &event)
            {
                RCLCPP_ERROR(this->get_logger(), "SUBSCRIBER::: INCOMPATIBLE QoS Triggered!");
                RCLCPP_INFO(this->get_logger(),
                    "Requested incompatible qos - total %d delta %d last_policy_kind: %d",
                    event.total_count, event.total_count_change, event.last_policy_kind);
            }

            void listener_callback(const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Data Received = '%s'", msg->data.c_str());
            }
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
            std::string durability;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberQoS>(argc, argv));
    rclcpp::shutdown();
    return 0;
}