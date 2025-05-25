#ifndef NODE_PARAMETERS_HPP
#define NODE_PARAMETERS_HPP
#include "rclcpp/rclcpp.hpp"
#include <string>

class NodeParameters
{
  public:
    // explicit NodeParameters(const std::shared_ptr<rclcpp::Node> & node): node_(node)
    explicit NodeParameters(rclcpp::Node* node) : node_(node)

    {
      RCLCPP_INFO(node_->get_logger(), "Initializing parameters for ZLAC8015D motor drivers");

      declare_parameters();
      load_parameters();
    }

    std::string connection_type;
    std::string modbus_port;
    int modbus_baudrate;
    std::string joint_state_topic;
    std::string wheel_rotation_topic;
    std::string zlac_status_topic;
    std::string reset_encoder_service;
    double joint_state_frequency;
    double zlac_status_frequency;
    int set_accel_time;
    int set_decel_time;
    int set_operation_mode;
    int set_stop_rpm;
    int set_clear_wheel_encoders;
    double ignore_small_speed_threshold;
    int max_rpm;
    double travel_in_one_rev;
    int cpr;
    double wheel_radius;

  private:
    // std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Node* node_;

    void declare_parameters()
    {
      node_->declare_parameter("connection_type", "rtu");
      node_->declare_parameter("modbus_port", "/dev/zlac_8015d");  // ttyUSB0   zlac_8015d
      node_->declare_parameter("modbus_baudrate", 115200);
      node_->declare_parameter("joint_state_topic", "/nhatbot/JointState");
      node_->declare_parameter("wheel_rotation_topic", "/nhatbot/wheel_rotational_vel");
      node_->declare_parameter("zlac_status_topic", "/nhatbot/zlac_status");
      node_->declare_parameter("reset_encoder_service", "/nhatbot/reset_feedback_position");
      node_->declare_parameter("joint_state_frequency", 0.1);
      node_->declare_parameter("zlac_status_frequency", 1.0);
      node_->declare_parameter("set_accel_time", 1000);
      node_->declare_parameter("set_decel_time", 1000);
      node_->declare_parameter("set_operation_mode", 3);
      node_->declare_parameter("set_stop_rpm", 0);
      node_->declare_parameter("set_clear_wheel_encoders", 3);
      node_->declare_parameter("ignore_small_speed_threshold", 0.5);
      node_->declare_parameter("max_rpm", 200);
      node_->declare_parameter("travel_in_one_rev", 0.336);
      node_->declare_parameter("cpr", 4096);
      node_->declare_parameter("wheel_radius", 0.0535);
    }

    void load_parameters()
    {
      try
      {
        node_->get_parameter("connection_type", connection_type);
        node_->get_parameter("modbus_port", modbus_port);
        node_->get_parameter("modbus_baudrate", modbus_baudrate);
        node_->get_parameter("joint_state_topic", joint_state_topic);
        node_->get_parameter("wheel_rotation_topic", wheel_rotation_topic);
        node_->get_parameter("zlac_status_topic", zlac_status_topic);
        node_->get_parameter("reset_encoder_service", reset_encoder_service);
        node_->get_parameter("joint_state_frequency", joint_state_frequency);
        node_->get_parameter("zlac_status_frequency", zlac_status_frequency);
        node_->get_parameter("set_accel_time", set_accel_time);
        node_->get_parameter("set_decel_time", set_decel_time);
        node_->get_parameter("set_operation_mode", set_operation_mode);
        node_->get_parameter("set_stop_rpm", set_stop_rpm);
        node_->get_parameter("set_clear_wheel_encoders", set_clear_wheel_encoders);
        node_->get_parameter("ignore_small_speed_threshold", ignore_small_speed_threshold);
        node_->get_parameter("max_rpm", max_rpm);
        node_->get_parameter("travel_in_one_rev", travel_in_one_rev);
        node_->get_parameter("cpr", cpr);
        node_->get_parameter("wheel_radius", wheel_radius);
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(node_->get_logger(), "Could not get parameters... setting defaults.");
        RCLCPP_WARN(node_->get_logger(), "Error: %s", e.what());
      }
    }
};
#endif 