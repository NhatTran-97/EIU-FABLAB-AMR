#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class JoyToTwistNode : public rclcpp::Node
{
public:
    JoyToTwistNode() : Node("joy_to_twist")
    {

        this->declare_parameter("linear_scale", 0.5);
        this->declare_parameter("angular_scale", 0.5);
        this->declare_parameter("deadman_button", 9);
        this->declare_parameter("deadzone_threshold", 0.01);

        linear_scale_ = this->get_parameter("linear_scale").as_double();
        angular_scale_ = this->get_parameter("angular_scale").as_double();
        deadman_button_ = this->get_parameter("deadman_button").as_int();
        deadzone_threshold_ = this->get_parameter("deadzone_threshold").as_double();

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToTwistNode::joy_callback, this, std::placeholders::_1));

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/input_joy/cmd_vel_stamped", 10);

        RCLCPP_INFO(this->get_logger(), "✅ joy_to_twist node is running...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.header.stamp = get_clock()->now();
        twist_stamped.header.frame_id = "base_link";

        // 🚀 Nếu không giữ nút deadman, dừng robot
        if (joy_msg->buttons[deadman_button_] == 0)
        {
            twist_stamped.twist.linear.x = 0.0;
            twist_stamped.twist.angular.z = 0.0;

            if (!is_zero_twist(last_twist_stamped_))
            {
                twist_publisher_->publish(twist_stamped);
                last_twist_stamped_ = twist_stamped;
            }

            return;
        }
        else
        {

            double linear_value = joy_msg->axes[1];  // Trục di chuyển tiến/lùi
            double angular_value = joy_msg->axes[2]; // Trục quay trái/phải

            // 🚫 Bỏ trôi nhẹ bằng deadzone
            if (std::abs(linear_value) < deadzone_threshold_)
            {
                linear_value = 0.0;
            }
            if (std::abs(angular_value) < deadzone_threshold_)
            {
                angular_value = 0.0;
            }

            twist_stamped.twist.linear.x = linear_value * linear_scale_;
            twist_stamped.twist.angular.z = angular_value * angular_scale_;
        }

        if (first_publish_ || twist_changed(twist_stamped, last_twist_stamped_))
        {
            twist_publisher_->publish(twist_stamped);
            last_twist_stamped_ = twist_stamped;
            first_publish_ = false;
        }
    }

    bool twist_changed(const geometry_msgs::msg::TwistStamped &a, const geometry_msgs::msg::TwistStamped &b)
    {
        return std::abs(a.twist.linear.x - b.twist.linear.x) > 1e-3 ||
               std::abs(a.twist.angular.z - b.twist.angular.z) > 1e-3;
    }
    bool is_zero_twist(const geometry_msgs::msg::TwistStamped &twist)
    {
        return std::abs(twist.twist.linear.x) < 1e-3 && std::abs(twist.twist.angular.z) < 1e-3;
    }

    double linear_scale_;
    double angular_scale_;
    int deadman_button_;
    double deadzone_threshold_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    geometry_msgs::msg::TwistStamped last_twist_stamped_;
    bool first_publish_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToTwistNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
