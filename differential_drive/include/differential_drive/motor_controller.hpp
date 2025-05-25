#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController();

private:

    void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);


    double wheel_radius_;
    double wheel_separation_;

    double linear_velocity_max_;
    double angular_velocity_max_;

    Eigen::Matrix2d speed_conversion_;


    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
};

#endif // MOTOR_CONTROLLER_HPP
