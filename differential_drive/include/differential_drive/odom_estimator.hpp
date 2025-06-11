#ifndef ODOM_ESTIMATOR_HPP
#define ODOM_ESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class OdomEstimator : public rclcpp::Node
{
public:
    OdomEstimator();
    ~OdomEstimator();

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update_odometry(double phi_left, double phi_right, double dt);
    void publish_odom();
    void reset_odom();
    void publish_transform();


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr transform_timer_;



    std::string odom_frame_;
    std::string base_frame_;
    
    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    
    
    nav_msgs::msg::Odometry odom_msg;
    double x_, y_, theta_;
    double left_wheel_prev_pos_, right_wheel_prev_pos_;
    double linear_filtered_, angular_filtered_;
    float correction_factor_;

    rclcpp::Time prev_time_;
    rclcpp::Time block_update_until_;

    


    // TF
    geometry_msgs::msg::TransformStamped transform_stamped_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    bool enable_tf_broadcast_;
    

    
    
 



};

#endif // ODOM_ESTIMATOR_HPP
