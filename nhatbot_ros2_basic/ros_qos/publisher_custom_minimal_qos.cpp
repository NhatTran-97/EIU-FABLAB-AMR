#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherQoS : public rclcpp::Node
{
    public:
        PublisherQoS(int &argc [[maybe_unused]] , char **argv) : Node("publisher_qos_obj")
        {
            reliability = argv[2];
            rclcpp::QoS qos_profile_publisher(10);

            if(reliability == "reliable")
            {
                qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            }
            else
            {
                qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            }
            rclcpp::PublisherOptions publisher_options;

            publisher_options.event_callbacks.incompatible_qos_callback = 
                            std::bind(&PublisherQoS::incompatible_qos_info_callback, this, _1);
            publisher_ = this->create_publisher<std_msgs::msg::String>("/qos_test", qos_profile_publisher,publisher_options);
            timer_ = this-> create_wall_timer(
                1000ms, std::bind(&PublisherQoS::timer_callback, this));
            msgs_id = 0;
        }
        void incompatible_qos_info_callback(rclcpp::QOSOfferedIncompatibleQoSInfo &event)
        {
            RCLCPP_ERROR(this->get_logger(),
                            "A subscriber is asking for an INCOMPATIBLE QoS Triggered!");
            RCLCPP_ERROR(
                this->get_logger(), "Offered incompatible qos - total %d delta %d last_policy_kind: %d",
                event.total_count, event.total_count_change, event.last_policy_kind);
        }
        
        void timer_callback()
        {
            auto msg = std_msgs::msg::String();
            auto current_time = this->now();
            auto current_time_s = current_time.seconds();
            auto current_time_ns = current_time.nanoseconds();
            time_str = 
                std::to_string(current_time_s) + "," + std::to_string(current_time_ns);
                dds_msg_str = std::to_string(msgs_id) + "," + time_str;
                msg.data = dds_msg_str;
                publisher_ -> publish(msg);
                RCLCPP_INFO(this->get_logger(), "Publishing: %s ", msg.data.c_str());
                ++msgs_id;
        }
    private:
        std::string reliability;
        int msgs_id;
        std::string time_str;
        std::string dds_msg_str;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherQoS>(argc, argv));
    rclcpp::shutdown();
    return 0;
}