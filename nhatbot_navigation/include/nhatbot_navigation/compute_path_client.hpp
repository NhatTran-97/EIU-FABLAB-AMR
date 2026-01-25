#ifndef COMPUTE_PATH_CLIENT__HPP_
#define COMPUTE_PATH_CLIENT__HPP_


#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace compute_path_client
{


class ComputePathClient : public rclcpp::Node
{
public:
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

explicit ComputePathClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
~ComputePathClient() override = default;


private:
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
};


} // namespace compute_path_client


#endif 