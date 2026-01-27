// #include <algorithm>

// #include "nav2_util/node_utils.hpp"
// #include "tf2/utils.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "nhatbot_controller/pure_pursuit_nav2_motion_planner.hpp"

// namespace nhatbot_controller
// {
// void PurePursuit::configure(
//   const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//   std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
//   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//   node_ = parent;

//   auto node = node_.lock();

//   costmap_ros_ = costmap_ros;
//   tf_buffer_ = tf_buffer;
//   plugin_name_ = name;
//   logger_ = node->get_logger();
//   clock_ = node->get_clock();

//   nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".look_ahead_distance",
//     rclcpp::ParameterValue(0.5));
//   nav2_util::declare_parameter_if_not_declared(
//     node, plugin_name_ + ".max_linear_velocity",rclcpp::ParameterValue(0.3));
//   nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_velocity",rclcpp::ParameterValue(1.0));

//   node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
//   node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
//   node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);

//   carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot", 1);
// }

// void PurePursuit::cleanup()
// {
//   RCLCPP_INFO(logger_, "Cleaning up PurePursuit");
//   carrot_pub_.reset();
// }

// void PurePursuit::activate()
// {
//   RCLCPP_INFO(logger_, "Activating PurePursuit");
//   carrot_pub_->on_activate();
// }

// void PurePursuit::deactivate()
// {
//   RCLCPP_INFO(logger_, "Deactivating PurePursuit");
//   carrot_pub_->on_deactivate();
// }

// geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
//                                                                     const geometry_msgs::msg::PoseStamped & robot_pose,
//                                                                     const geometry_msgs::msg::Twist &,
//                                                                     nav2_core::GoalChecker *)
// {
//   auto node = node_.lock();
//   geometry_msgs::msg::TwistStamped cmd_vel;
//   cmd_vel.header.frame_id = robot_pose.header.frame_id;

//   if(global_plan_.poses.empty()){
//     RCLCPP_ERROR(logger_, "Empty Plan!");
//     return cmd_vel;
//   }

//   if(!transformPlan(robot_pose.header.frame_id)){
//     RCLCPP_ERROR(logger_, "Unable to transform Plan in robot's frame");
//     return cmd_vel;
//   }

//   auto carrot_pose = getCarrotPose(robot_pose);
//   carrot_pub_->publish(carrot_pose);
        
//   // Calculate the curvature to the look-ahead point
//   tf2::Transform carrot_pose_robot_tf, robot_tf, carrot_pose_tf;
//   tf2::fromMsg(robot_pose.pose, robot_tf);
//   tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
//   carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;
//   tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);
//   double curvature = getCurvature(carrot_pose.pose);
        
//   // Create and publish the velocity command
//   cmd_vel.twist.linear.x = max_linear_velocity_;
//   cmd_vel.twist.angular.z = curvature * max_angular_velocity_;

//   return cmd_vel;
// }

// void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
// {
//   RCLCPP_INFO_STREAM(logger_, "Path received with " << path.poses.size() << " poses");
//   RCLCPP_INFO_STREAM(logger_, "Path frame " << path.header.frame_id);
//   global_plan_ = path;
// }

// void PurePursuit::setSpeedLimit(const double &, const bool &)
// {

// }

// geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
// {
//   geometry_msgs::msg::PoseStamped carrot_pose = global_plan_.poses.back();
//   for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) 
//   {
//     double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
//     double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
//     double distance = std::sqrt(dx * dx + dy * dy);
//     if(distance > look_ahead_distance_)
//     {
//       carrot_pose = *pose_it;
//     } else 
//     {
//       break;
//     }
//   }
//   return carrot_pose;
// }

// double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
// {
//   const double carrot_dist =
//   (carrot_pose.position.x * carrot_pose.position.x) +
//   (carrot_pose.position.y * carrot_pose.position.y);
    
//   // Find curvature of circle (k = 1 / R)
//   if (carrot_dist > 0.001) 
//   {
//     return 2.0 * carrot_pose.position.y / carrot_dist;
//   } else 
//   {
//     return 0.0;
//   }
// }

// bool PurePursuit::transformPlan(const std::string & frame)
// {
//   if(global_plan_.header.frame_id == frame)
//   {
//     return true;
//   }
//   geometry_msgs::msg::TransformStamped transform;
//   try
//   {
//     transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
//   } 
//   catch (tf2::ExtrapolationException & ex) 
//   {
//     RCLCPP_ERROR_STREAM(logger_, "Couldn't transform plan from frame " <<
//       global_plan_.header.frame_id << " to frame " << frame);
//     return false;
//   }
//   for(auto & pose : global_plan_.poses)
//   {
//     tf2::doTransform(pose, pose, transform);
//   }
//   global_plan_.header.frame_id = frame;
//   return true;
// }

// }  

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nhatbot_controller::PurePursuit, nav2_core::Controller)



#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nhatbot_controller/pure_pursuit_nav2_motion_planner.hpp"

namespace nhatbot_controller
{

void PurePursuit::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // ==============================
  // Parameters
  // ==============================
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(0.6));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.35));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.05));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.2));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_gain", rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".slow_down_distance", rclcpp::ParameterValue(0.8));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_factor", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".min_linear_velocity", min_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".curvature_gain", curvature_gain_);
  node->get_parameter(plugin_name_ + ".slow_down_distance", slow_down_distance_);
  node->get_parameter(plugin_name_ + ".cost_scaling_factor", cost_scaling_factor_);

  carrot_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pure_pursuit/carrot", 1);
}

void PurePursuit::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up Regulated Pure Pursuit");
  carrot_pub_.reset();
}

void PurePursuit::activate()
{
  RCLCPP_INFO(logger_, "Activating Regulated Pure Pursuit");
  carrot_pub_->on_activate();
}

void PurePursuit::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating Regulated Pure Pursuit");
  carrot_pub_->on_deactivate();
}

// =====================================================
// Main control loop
// =====================================================
geometry_msgs::msg::TwistStamped
PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = robot_pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "Global plan is empty");
    return cmd_vel;
  }

  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "Failed to transform global plan");
    return cmd_vel;
  }

  // ==============================
  // Get carrot pose
  // ==============================
  auto carrot_pose = getCarrotPose(robot_pose);
  carrot_pub_->publish(carrot_pose);

  // Transform carrot into robot frame
  tf2::Transform robot_tf, carrot_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_tf);

  tf2::Transform carrot_in_robot = robot_tf.inverse() * carrot_tf;

  geometry_msgs::msg::Pose carrot_pose_robot;
  tf2::toMsg(carrot_in_robot, carrot_pose_robot);

  // ==============================
  // Curvature
  // ==============================
  double curvature = getCurvature(carrot_pose_robot);
  double abs_curvature = std::fabs(curvature);

  // ==============================
  // Linear velocity regulation
  // ==============================
  double linear_vel =
    max_linear_velocity_ /
    (1.0 + curvature_gain_ * abs_curvature);

  // Slow down near goal
  auto & goal = global_plan_.poses.back().pose.position;
  double dx = goal.x - robot_pose.pose.position.x;
  double dy = goal.y - robot_pose.pose.position.y;
  double dist_to_goal = std::hypot(dx, dy);

  if (dist_to_goal < slow_down_distance_) {
    linear_vel *= (dist_to_goal / slow_down_distance_);
  }

  // Costmap regulation
  unsigned int mx, my;
  if (costmap_->worldToMap(
        robot_pose.pose.position.x,
        robot_pose.pose.position.y,
        mx, my))
  {
    unsigned char cost = costmap_->getCost(mx, my);
    double cost_scale =
      1.0 - cost_scaling_factor_ * (static_cast<double>(cost) / 255.0);

    linear_vel *= std::clamp(cost_scale, 0.3, 1.0);
  }

  linear_vel = std::clamp(
    linear_vel,
    min_linear_velocity_,
    max_linear_velocity_);

  // ==============================
  // Angular velocity
  // ==============================
  double angular_vel = curvature * linear_vel;
  angular_vel = std::clamp(
    angular_vel,
    -max_angular_velocity_,
    max_angular_velocity_);

  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

// =====================================================
// Path handling
// =====================================================
void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(
    logger_,
    "Received path with " << path.poses.size() << " poses");

  global_plan_ = path;
}

void PurePursuit::setSpeedLimit(const double &, const bool &) {}

// =====================================================
// Helpers
// =====================================================
geometry_msgs::msg::PoseStamped
PurePursuit::getCarrotPose(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped carrot_pose =
    global_plan_.poses.back();

  for (auto it = global_plan_.poses.rbegin();
       it != global_plan_.poses.rend(); ++it)
  {
    double dx = it->pose.position.x - robot_pose.pose.position.x;
    double dy = it->pose.position.y - robot_pose.pose.position.y;

    if (std::hypot(dx, dy) > look_ahead_distance_) {
      carrot_pose = *it;
      break;
    }
  }
  return carrot_pose;
}

double PurePursuit::getCurvature(
  const geometry_msgs::msg::Pose & carrot_pose)
{
  double dist_sq =
    carrot_pose.position.x * carrot_pose.position.x +
    carrot_pose.position.y * carrot_pose.position.y;

  if (dist_sq < 1e-6) {
    return 0.0;
  }

  return 2.0 * carrot_pose.position.y / dist_sq;
}

bool PurePursuit::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame) {
    return true;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      frame,
      global_plan_.header.frame_id,
      tf2::TimePointZero);
  }
  catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "%s", ex.what());
    return false;
  }

  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, tf);
  }

  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace nhatbot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nhatbot_controller::PurePursuit,
  nav2_core::Controller)
