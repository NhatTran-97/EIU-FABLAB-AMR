#include <queue>
#include "nhatbot_planner/a_star_nav2_smoother.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>


#include <queue>
#include <vector>



namespace nhatbot_planning
{

    void AStarPlanner_Smoother::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");


        // if(!smooth_client_->wait_for_action_server(std::chrono::seconds(3)))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        // }
        
    }
    
    void AStarPlanner_Smoother::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up %s of type AStarPlanner", name_.c_str());

    }
    void AStarPlanner_Smoother::activate()
    {
         RCLCPP_INFO(node_->get_logger(), "Activating %s of type AStarPlanner", name_.c_str());


    }
    void AStarPlanner_Smoother::deactivate()
    {
          RCLCPP_INFO(node_->get_logger(), "Deactivating %s of type AStarPlanner", name_.c_str());

    }



    nav_msgs::msg::Path  AStarPlanner_Smoother::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                    const geometry_msgs::msg::PoseStamped &goal)

{
    std::vector<std::pair<int, int>> explore_directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
    std::vector<GraphNode> visited_nodes;

    GraphNode start_node = worldToGrid(start.pose);
    GraphNode goal_node = worldToGrid(goal.pose);
    start_node.heuristic = manhattanDistance(start_node, goal_node);
    pending_nodes.push(start_node);

    GraphNode active_node;
    while (!pending_nodes.empty() && rclcpp::ok()) {
        active_node = pending_nodes.top();
        pending_nodes.pop();

        // Goal found!
        if(worldToGrid(goal.pose) == active_node){
            break;
        }

        // Explore neighbors
        for (const auto & dir : explore_directions) {
            GraphNode new_node = active_node + dir;
            // Check if the new position is within bounds and not an obstacle
            if (std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() &&
                poseOnMap(new_node) && costmap_->getCost(new_node.x, new_node.y) < 99 ) 
                {
                // If the node is not visited, add it to the queue
                new_node.cost = active_node.cost + 1 + costmap_->getCost(new_node.x, new_node.y);
                new_node.heuristic = manhattanDistance(new_node, goal_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;

    while(active_node.prev && rclcpp::ok()) 
    {
        geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = global_frame_;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);
        active_node = *active_node.prev;
    }
    std::reverse(path.poses.begin(), path.poses.end());

    if(smooth_client_->action_server_is_ready())
    {
        nav2_msgs::action::SmoothPath::Goal path_smooth;
        path_smooth.path = path;
        path_smooth.check_for_collisions = false;
        path_smooth.smoother_id = "simple_smoother";
        path_smooth.max_smoothing_duration.sec = 10;
        auto future = smooth_client_->async_send_goal(path_smooth);

        if(future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
        {
        auto goal_handle = future.get();
        if(goal_handle)
        {
            auto result_future = smooth_client_->async_get_result(goal_handle);
            if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
            {
            auto result_path = result_future.get();
            if(result_path.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                path = result_path.result->path;
            }
            }
        }
        }
    }
    


    return path;
}

bool AStarPlanner_Smoother::poseOnMap(const GraphNode & node)
{
    return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
        node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
}

GraphNode AStarPlanner_Smoother::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlanner_Smoother::gridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    return pose;
}

unsigned int AStarPlanner_Smoother::poseToCell(const GraphNode & node)
{
    return costmap_->getSizeInCellsX() * node.y + node.x;
}

double AStarPlanner_Smoother::manhattanDistance(const GraphNode & node, const GraphNode &goal_node)
{
    return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
}
}  


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_planning::AStarPlanner_Smoother, nav2_core::GlobalPlanner)


