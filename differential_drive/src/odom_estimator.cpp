#include "differential_drive/odom_estimator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "rmw/qos_profiles.h"
#include <queue>
#include "rclcpp/qos.hpp"

// Constructor: initialize parameters, subscriptions, publishers, and transform broadcaster
OdomEstimator::OdomEstimator() : Node("odom_estimator_node"),
                                  odom_frame_("odom"), base_frame_("base_link"),
                                 x_(0.0), y_(0.0), theta_(0.0),
                                 left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0),
                                 linear_filtered_(0.0), angular_filtered_(0.0), correction_factor_(1.0), 
                                 enable_tf_broadcast_(true)
                                 
{
    // Declare and retrieve parameters for the robot's physical properties

    this->declare_parameter("odom_frame", odom_frame_);
    this->declare_parameter("base_frame", base_frame_);

    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);


    this->declare_parameter("wheel_radius", 0.0535);
    this->declare_parameter("wheel_separation", 0.45);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_separation", wheel_separation_);

    this->declare_parameter("enable_tf_broadcast", true);
    this->get_parameter("enable_tf_broadcast", enable_tf_broadcast_);

    this->declare_parameter("correction_factor", 1.0);
    this->get_parameter("correction_factor", correction_factor_);
    

    // Define a QoS (Quality of Service) profile for subscriptions and publishers
  

    rclcpp::QoS qos_wheel_jointState(rclcpp::KeepLast(10));
    qos_wheel_jointState.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_wheel_jointState.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


    // Subscribe to joint state topic from zlac_driver_node
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>( "nhatbot/JointState", qos_wheel_jointState, 
                                                                        std::bind(&OdomEstimator::joint_callback, this, std::placeholders::_1));
    
    // Create publisher for Odometry messages
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("nhatbot/odom", qos_wheel_jointState);

    transform_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomEstimator::publish_transform, this) );


    // Initialize Odometry message with static frame information
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Set up TF broadcaster and static frame relationship
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_link";

    // Initialize time for computing delta t in updates
    prev_time_ = get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "✅ odom_estimator_node started!");
}

// Destructor: clean up publisher/subscribers
OdomEstimator::~OdomEstimator()
{
    RCLCPP_INFO(this->get_logger(), "🛑 odom_estimator_node is stopped...");
    odom_pub_.reset();
    joint_sub_.reset();
    transform_broadcaster_.reset();
}

// Callback for encoder data (wheel position data)
void OdomEstimator::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    /*
        Implements the inverse differential kinematic model.
        Given the wheel positions, it computes their angular velocities, estimates the robot's linear and angular velocities, updates the robot's pose (x, y, theta),
        and publishes Odometry and TF information.
    */

    // Ensure the message contains exactly two joint positions (left and right wheel)
    if (msg->position.size() != 2)
    {
        return;
    }
    
    // Use timestamp from message, or fallback if invalid
    rclcpp::Time current_time = msg->header.stamp;

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) 
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ JointState has no timestamp");
        return;
    
    } else 
    {
        current_time = msg->header.stamp; 
    }


    // Compute time delta between callbacks, avoid zero

    double dt = std::max((current_time - prev_time_).seconds(), 1e-6);
    prev_time_ = current_time;

    // If both encoder values reset to zero (likely after reboot/reset) => reset odometry

    if (msg->position[0] == 0.0 && msg->position[1] == 0.0)
    {
        // If previous positions were not zero, this indicates an actual reset
        if (left_wheel_prev_pos_ != 0.0 || right_wheel_prev_pos_ != 0.0)
        {
            reset_odom();
        }
        return;
    }

     // Compute wheel displacement (delta position) since last update
    double dp_left = (msg->position[0] - left_wheel_prev_pos_) * correction_factor_;
    double dp_right = (msg->position[1] - right_wheel_prev_pos_) * correction_factor_;

    // Ignore very small movements (likely noise)
    if (std::abs(dp_left) < 0.0005 && std::abs(dp_right) < 0.0005)
        return;

    // Update previous wheel positions for next iteration
    left_wheel_prev_pos_ = msg->position[0];
    right_wheel_prev_pos_ = msg->position[1];

 
    // Compute rotational speeds (rad/s) of each wheel
    double phi_left = dp_left / dt;
    double phi_right = dp_right / dt;

    // Estimate odometry based on these wheel velocities
    this->update_odometry(phi_left, phi_right, dt);
}

// Update robot pose using wheel angular velocities and differential-drive kinematics
void OdomEstimator::update_odometry(double phi_left, double phi_right, double dt)
{
    // Compute the linear and angular velocity
    double linear_velocity = wheel_radius_ * (phi_right + phi_left) / 2.0;
    double angular_velocity = wheel_radius_ * (phi_left - phi_right ) / wheel_separation_;

    // Low-pass filter and angular velocity of the robot
    double alpha = 0.2;
    linear_filtered_ = alpha * linear_velocity + (1 - alpha) * linear_filtered_;
    angular_filtered_ = alpha * angular_velocity + (1 - alpha) * angular_filtered_;

    // Zero out tiny values to avoid drift
    if (std::abs(linear_filtered_) < 0.001)
        linear_filtered_ = 0.0;
    if (std::abs(angular_filtered_) < 0.001)
        angular_filtered_ = 0.0;

    // // Calculate the position increment
    // double d_s = (wheel_radius_ * phi_right * dt + wheel_radius_ * phi_left * dt) / 2.0;
    // double d_theta = (wheel_radius_ * phi_right * dt - wheel_radius_ * phi_left * dt) / wheel_separation_;


    // Update robot pose (x, y, theta) using velocities and time delta
    x_ += linear_filtered_ * dt * std::cos(theta_);
    y_ += linear_filtered_ * dt * std::sin(theta_);
    theta_ += angular_filtered_ * dt;

    // Normalize theta to [-π, π]
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

    // Publish both odometry and transform
    this->publish_odom();

    
}

// void OdomEstimator::publish_transform()
// {
//     if(enable_tf_broadcast_)
//     {

//         this->publish_transform();
    
//     }
// }



void OdomEstimator::publish_transform()
{
    if (!enable_tf_broadcast_) return;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    q.normalize(); 

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.translation.z = 0.0;

    // transform_stamped_.transform.rotation = tf2::toMsg(q);
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();

    if (rclcpp::ok())
    {
        transform_broadcaster_->sendTransform(transform_stamped_);
    }

    
}

// Publish nav_msgs/Odometry with current pose and twist
void OdomEstimator::publish_odom()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    q.normalize(); 

    odom_msg.header.stamp = get_clock()->now();
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;

    // Assign quaternion orientation
    // odom_msg.pose.pose.orientation = tf2::toMsg(q);
   
    // odom_msg.pose.pose.orientation.x = 0.0; //q.getX();
    // odom_msg.pose.pose.orientation.y = 0.0; //q.getY();
    // odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0); //q.getZ();
    // odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);  //q.getW();

    odom_msg.pose.pose.orientation.x = q.getX();
    odom_msg.pose.pose.orientation.y = q.getY();
    odom_msg.pose.pose.orientation.z = q.getZ();
    odom_msg.pose.pose.orientation.w = q.getW();

    // Position uncertainty
    odom_msg.pose.covariance[0] = 0.2; ///< x
    odom_msg.pose.covariance[7] = 0.2; ///< y
    odom_msg.pose.covariance[35] = 0.4;  ///< yaw

    // Assign linear and angular velocity
    odom_msg.twist.twist.linear.x = linear_filtered_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_filtered_;
    
    odom_pub_->publish(odom_msg);

}

// Broadcast transform from "odom" -> "base_link" using the current pose




// Reset the odoemtry estimate (called if encoder reset detected)
void OdomEstimator::reset_odom()
{
    x_ = y_ = theta_ = 0.0;
    left_wheel_prev_pos_ = right_wheel_prev_pos_ = 0.0;
    // prev_time_ = get_clock()->now();
    linear_filtered_ = 0.0;
    angular_filtered_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "⚡ Encoder reset detected! Odom reset automatically.");

    publish_odom(); // Publish reset state immediately
}


// Main entry point for ROS2 node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomEstimator>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "❌ Error runtime: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "✅ odom_estimator_node shutdown complete");

    node.reset(); // Clean up

    if (rclcpp::ok()) // just shutdown if ros is running
    {
        rclcpp::shutdown();
    }

    return 0;
}
