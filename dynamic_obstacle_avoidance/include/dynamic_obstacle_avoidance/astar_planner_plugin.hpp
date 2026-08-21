#ifndef DYNAMIC_OBSTACLE_AVOIDANCE__ASTAR_PLANNER_PLUGIN_HPP_
#define DYNAMIC_OBSTACLE_AVOIDANCE__ASTAR_PLANNER_PLUGIN_HPP_

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "dynamic_obstacle_avoidance/astar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace dynamic_obstacle_avoidance {

class AStarPlannerPlugin : public nav2_core::GlobalPlanner {
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal,
                                   std::function<bool()> cancel_checker) override;

private:
    std::unique_ptr<AStarPlanner> planner_core_;
    nav2_costmap_2d::Costmap2DROS* costmap_ros_;
    std::string global_frame_;
    rclcpp::Clock::SharedPtr clock_;
};

} // namespace dynamic_obstacle_avoidance
#endif
