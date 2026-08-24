#include "dynamic_obstacle_avoidance/rrt_planner_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamic_obstacle_avoidance::RRTPlannerPlugin, nav2_core::GlobalPlanner)

namespace dynamic_obstacle_avoidance {

void RRTPlannerPlugin::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                   std::string name,
                                   std::shared_ptr<tf2_ros::Buffer> tf,
                                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) 
{
    (void)parent;
    (void)name;
    (void)tf;
    
    planner_core_ = std::make_unique<RRTPlanner>();
    costmap_ros_ = costmap_ros.get();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    
    // Get node from parent weak_ptr to get clock
    auto node = parent.lock();
    if (node) {
        clock_ = node->get_clock();
    } else {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }
}

void RRTPlannerPlugin::activate() {}
void RRTPlannerPlugin::deactivate() {}
void RRTPlannerPlugin::cleanup() { planner_core_.reset(); }

nav_msgs::msg::Path RRTPlannerPlugin::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                                   const geometry_msgs::msg::PoseStamped& goal,
                                                   std::function<bool()> cancel_checker) 
{
    (void)cancel_checker;

    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = clock_->now();

    if (!costmap_ros_) {
        return path;
    }

    auto costmap = costmap_ros_->getCostmap();
    if (!costmap) {
        return path;
    }

    int w = static_cast<int>(costmap->getSizeInCellsX());
    int h = static_cast<int>(costmap->getSizeInCellsY());

    // Convert start and goal world coordinates to map grid coordinates
    unsigned int start_x = 0, start_y = 0;
    unsigned int goal_x = 0, goal_y = 0;

    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        return path;
    }

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        return path;
    }

    std::vector<int> grid(w * h);
    const unsigned char* char_map = costmap->getCharMap();
    for (int i = 0; i < w * h; ++i) {
        grid[i] = static_cast<int>(char_map[i]);
    }

    auto core_path = planner_core_->plan(
        grid, w, h, 
        {static_cast<int>(start_x), static_cast<int>(start_y)}, 
        {static_cast<int>(goal_x), static_cast<int>(goal_y)}
    );

    // Convert planned map grid points back to world coordinates
    for (const auto& p : core_path) {
        double wx = 0.0, wy = 0.0;
        costmap->mapToWorld(static_cast<unsigned int>(p.x), static_cast<unsigned int>(p.y), wx, wy);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = global_frame_;
        ps.header.stamp = clock_->now();
        ps.pose.position.x = wx;
        ps.pose.position.y = wy;
        ps.pose.position.z = 0.0;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
    }

    // Preserve orientation for final goal pose if path is non-empty
    if (!path.poses.empty()) {
        path.poses.back().pose.orientation = goal.pose.orientation;
    }

    return path;
}

} // namespace dynamic_obstacle_avoidance
