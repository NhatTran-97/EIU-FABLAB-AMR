#include <angles/angles.h>
#include <cmath>
#include <nhatbot_controller/lqr_node.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

input input_old = input(0, 0);

LqrNode::LqrNode(): Node("LqrNode"), dt_(0.03), tolerance(0.5), end_controller(false),
      max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false)
{
  // 1️⃣ TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 2️⃣ Sub/Pub
  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/a_star/path", 10, std::bind(&LqrNode::pathCallback, this, std::placeholders::_1));

  control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  control_loop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&LqrNode::controlLoopCallback, this));

  // 3️⃣ LQR setup
  Q_ << 0.8, 0, 0, 0, 0.8, 0, 0, 0, 0.8;
  R_ << 0.8, 0, 0, 0.8;
  lqr_ = std::make_unique<LQR>(Q_, R_, 100);

  RCLCPP_INFO(this->get_logger(), "LQR Node initialized");
}

void LqrNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Received empty path!");
    return;
  }

  // 4️⃣ Transform path to odom frame
  nav_msgs::msg::Path transformed_path;
  transformed_path.header.frame_id = "odom";

  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform path: %s", ex.what());
    return;
  }

  transformed_path.poses.reserve(msg->poses.size());
  for (auto pose : msg->poses)
  {
    tf2::doTransform(pose, pose, transformStamped);
    transformed_path.poses.push_back(pose);
  }

  // 5️⃣ Convert to internal waypoint format
  waypoints_.clear();
  for (size_t i = 0; i < transformed_path.poses.size(); ++i)
  {
    double x = transformed_path.poses[i].pose.position.x;
    double y = transformed_path.poses[i].pose.position.y;

    double yaw = tf2::getYaw(transformed_path.poses[i].pose.orientation);
    waypoints_.push_back(State(x, y, yaw));
  }

  optimiseHeading(waypoints_);
  current_waypoint = 0;
  end_controller = false;
  RCLCPP_INFO(this->get_logger(), "Received and transformed path with %zu waypoints", waypoints_.size());
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  actual_state_ = State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  odom_received_ = true;
}

void LqrNode::publishVelocity(double v, double w)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  control_input_ = input(v, w);
  input_old = input(v, w);
  control_input_pub_->publish(msg);
}

void LqrNode::optimiseHeading(std::vector<State> &waypoints)
{
  for (size_t i = 0; i < waypoints.size() - 1; ++i)
  {
    double dx = waypoints[i + 1].x - waypoints[i].x;
    double dy = waypoints[i + 1].y - waypoints[i].y;
    waypoints[i].theta = std::atan2(dy, dx);
  }
  waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
}

void LqrNode::controlLoopCallback()
{
  if (!odom_received_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry...");
    return;
  }

  if (waypoints_.empty())
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for path...");
    return;
  }

  if (end_controller)
  {
    publishVelocity(0.0, 0.0);
    return;
  }

  // 6️⃣ Get current waypoint
  State desired_state = waypoints_[current_waypoint];
  Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y, actual_state_.theta);
  Eigen::Vector3d x_desired(desired_state.x, desired_state.y, desired_state.theta);
  state_error_ = x_actual - x_desired;

  auto A = lqr_->getA(actual_state_.theta, control_input_.v, dt_);
  auto B = lqr_->getB(actual_state_.theta, dt_);
  lqr_->updateMatrices(A, B);
  lqr_->computeRiccati(B, A);
  auto u = lqr_->computeOptimalInput(state_error_);

  publishVelocity(
      std::clamp(u(0), -max_linear_velocity, max_linear_velocity),
      std::clamp(u(1), -max_angular_velocity, max_angular_velocity));

  if (state_error_.norm() < tolerance)
  {
    current_waypoint++;
    if (current_waypoint >= waypoints_.size())
    {
      end_controller = true;
      publishVelocity(0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "Goal reached! Path completed.");
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LqrNode>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
