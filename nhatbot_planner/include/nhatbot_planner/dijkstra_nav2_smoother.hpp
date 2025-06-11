#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

namespace nhatbot_planning
{
struct GraphNode
{
    int x;
    int y;
    int cost;
    std::shared_ptr<GraphNode> prev;

    GraphNode() : GraphNode(0,0) {}

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0){}

    bool operator>(const GraphNode & other) const { 
        return cost > other.cost;
    }

    bool operator==(const GraphNode & other) const {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(std::pair<int, int> const & other) {
        GraphNode res(x + other.first, y + other.second);
        return res;
    }
};

class DijkstraPlanner_Smoother : public nav2_core::GlobalPlanner
{
public:
    DijkstraPlanner_Smoother() = default;
    ~DijkstraPlanner_Smoother() = default;

    /*
    The configure function takes as input all the information that we need so that the planner,
    so the planner needs in order to calculate the path, namely it takes a node (our ros2 node, name of the plugin, TF buffer)
    => We can use in order to calculate the current position of the robot
    So we perfectly know apart from the TF, apart from the name and apart from the node. The planner server also takes care of initializing the costmap, namely the global cost map
    override: indicates that this configure function is declared in the base class and here we are defining it
    */
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
                         std::string name, 
                         std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path  createPlan(const geometry_msgs::msg::PoseStamped &start,
                                    const geometry_msgs::msg::PoseStamped &goal) override;

private:
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    // nav_msgs::msg::OccupancyGrid visited_map_;

    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);


    /*
    Simply we are not going to initialize any new TF2 ros buffer, but simply we are setting this one, we are going to set this one equal to one
    */

    std::shared_ptr<tf2_ros::Buffer> tf_;

    /*
    We are not going to create any new node, but simply we are going to set this one equal to the one that we are received from the planner server 
    */

    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_, name_;

    rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    bool poseOnMap(const GraphNode & node);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);

    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);

    unsigned int poseToCell(const GraphNode & node);
};
}  // namespace bumperbot_planning

#endif // DIJKSTRA_PLANNER_HPP