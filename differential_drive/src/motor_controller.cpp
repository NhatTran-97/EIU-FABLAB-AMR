#include "differential_drive/motor_controller.hpp"
#include <cmath>
#include <algorithm>
#include "rmw/qos_profiles.h"
#include <queue>
#include "rclcpp/qos.hpp"

// Constructor
MotorController::MotorController() : Node("motor_controller_node"),  wheel_radius_(0.0535), wheel_separation_(0.45), 
                                                                linear_velocity_max_(0.5),angular_velocity_max_(0.5)
{
    // Declare and retrieve robot physical parameters

    this->declare_parameter("wheel_radius", 0.0535);
    this->declare_parameter("wheel_separation", 0.45);

    
    this->declare_parameter("linear_velocity_max", 0.5);
     this->declare_parameter("angular_velocity_max", 0.5);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_separation", wheel_separation_);

    this->get_parameter("linear_velocity_max", linear_velocity_max_);
    this->get_parameter("angular_velocity_max", angular_velocity_max_);

    RCLCPP_INFO(this->get_logger(), "Using wheel_radius: %.4f, wheel_separation: %.4f", wheel_radius_, wheel_separation_);

    // Define a QoS (Quality of Service) profile for subscriptions and publishers

    rclcpp::QoS qos(rclcpp::KeepLast(10));
  
    rclcpp::QoS qos_cmd(rclcpp::KeepLast(1));
    qos_cmd.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_cmd.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    //  Create a subscriber to velocity commands (TwistStamped from joystick or controller)

    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("nhatbot/cmd_vel", qos, 
                                                                            std::bind(&MotorController::vel_callback, this, std::placeholders::_1));

    // Create a publisher to send computed wheel rotational velocities (rad/s)
    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nhatbot/wheel_rotational_vel", qos_cmd);


    // Build the conversion matrix from wheel angular velocities (phi_r, phi_l) to robot velocities (v, omega)
    speed_conversion_ << 
    wheel_radius_ / 2.0,                  wheel_radius_ / 2.0,
    wheel_radius_ / wheel_separation_, -wheel_radius_ / wheel_separation_;
    

    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);
}

// Destructor
MotorController::~MotorController()
{
    RCLCPP_INFO(this->get_logger(), "🛑 MotorController node is stopping...");
}

// Callback function to handle velocity commands
void MotorController::vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    /*
     Implements the differential kinematic model
     Given linear (v) and angular (w) velocity, compute individual wheel angular speeds. 
     Uses a pre-defined speed_conversion_ matrix for flexible anf clarity
    */

    // Validate input to avoid NaN or infinite values
    if (std::isnan(msg->twist.linear.x) || std::isinf(msg->twist.linear.x) ||
        std::isnan(msg->twist.angular.z) || std::isinf(msg->twist.angular.z))
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Invalid cmd_vel: NaN or Inf detected. Ignoring this command.");
        return;
    }

    // Clamp input velocities to a safe range to avoid sending extreme values to motors
    double linear_x = std::clamp(msg->twist.linear.x, -linear_velocity_max_, linear_velocity_max_);
    double angular_z = std::clamp(msg->twist.angular.z, -angular_velocity_max_, angular_velocity_max_);  // adjust as needed

   // Construct robot velocity vector (v, ω)
    Eigen::Vector2d robot_speed(linear_x, angular_z);

    // Compute wheel angular velocities using inverse of the conversion matrix
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    
    // Prepare message to send to motor driver
    std_msgs::msg::Float64MultiArray wheel_speed_msg;

    wheel_speed_msg.data = {
        static_cast<float>(wheel_speed.coeff(0)),  // left wheel
        static_cast<float>(wheel_speed.coeff(1))   // right wheel 
    };

    // Safety check to ensure message has exactly 2 elements (left, right)
    if (wheel_speed_msg.data.size() != 2)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ wheel_speed_msg must have exactly 2 elements (left, right), but got %ld", wheel_speed_msg.data.size());
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "🚗 Wheel Speeds [rad/s] → Left: %.3f | Right: %.3f",
    //         wheel_speed_msg.data[0], wheel_speed_msg.data[1]);

    // Publish computed wheel speeds
    wheel_cmd_pub_->publish(wheel_speed_msg);

}


// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "❌ Runtime error: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "✅ MotorController Node has shut down cleanly.");

    if (rclcpp::ok()) 
    {
        rclcpp::shutdown();
    }

    return 0;
}
