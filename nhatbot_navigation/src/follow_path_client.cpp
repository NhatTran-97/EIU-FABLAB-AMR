#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using namespace std::chrono_literals;

class FollowPathClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathClient(): Node("follow_path_client")
  {
    client_ = rclcpp_action::create_client<FollowPath>(this,"/follow_path");

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&FollowPathClient::pathCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FollowPathClient started");
  }

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (path->poses.empty()) 
    {
      RCLCPP_WARN(get_logger(), "Received empty path, ignoring");
      return;
    }

    if (!client_->wait_for_action_server(2s)) 
    {
      RCLCPP_ERROR(get_logger(), "FollowPath action server not available");
      return;
    }

    FollowPath::Goal goal;
    goal.path = *path;

    /*
      ["FollowPath_PDMotionPlanner", "FollowPath_PurePursuit","FollowPath_DWB", "FollowPath_RegulatedPurePursuit", "FollowPath_WindowDynamicPurePursuit"]
    */
    goal.controller_id = "FollowPath_DWB";  
    goal.goal_checker_id = "general_goal_checker";

    
    RCLCPP_INFO(get_logger(), "Sending path (%zu poses) to controller [%s]", path->poses.size(), goal.controller_id.c_str());

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

    send_goal_options.result_callback = [](const GoalHandleFollowPath::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
        {
          RCLCPP_INFO(rclcpp::get_logger("follow_path_result"),"Path following succeeded");
        } 
        else 
        {
          RCLCPP_ERROR(rclcpp::get_logger("follow_path_result"), "Path following failed with code %d", static_cast<int>(result.code));
        }
      };

    client_->async_send_goal(goal, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowPathClient>());
  rclcpp::shutdown();
  return 0;
}
