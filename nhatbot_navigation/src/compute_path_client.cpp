#include "nhatbot_navigation/compute_path_client.hpp"

#include <chrono>
#include <utility>


using namespace std::chrono_literals;

namespace compute_path_client
{

ComputePathClient::ComputePathClient(const rclcpp::NodeOptions & options): Node("compute_path_client", options)
{
    client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");

    RCLCPP_INFO(get_logger(), "ComputePathClient started, waiting for planner...");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();




    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", qos, std::bind(&ComputePathClient::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ComputePathClient ready");
}

void ComputePathClient::goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!client_->action_server_is_ready()) 
    {
        RCLCPP_WARN(get_logger(),"Planner action server not ready yet, ignoring goal");
        return;
    }

    /*
    ["GridBased_DijkstraPlanner_Smoother", "GridBased_AStarPlanner_Smoother", "GridBased_SmacPlanner2D_nav2", 
    "GridBased_SmacPlannerHybrid_nav2", GridBased_ThetaStarPlanner_nav2]
    */
    ComputePathToPose::Goal goal;
    goal.goal = *msg;
    goal.planner_id = "GridBased_ThetaStarPlanner_nav2";

    auto options =
        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

    options.result_callback = [this](const auto & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Path computed with %zu poses",
                    result.result->path.poses.size()
                );
            } 
            else 
            {
                RCLCPP_ERROR(get_logger(), "Path computation failed");
            }
        };

    client_->async_send_goal(goal, options);
}


} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<compute_path_client::ComputePathClient>(
            rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




