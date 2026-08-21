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

    // 1. Get Costmap Data
    auto costmap = costmap_ros_->getCostmap();
    int w = static_cast<int>(costmap->getSizeInCellsX());
    int h = static_cast<int>(costmap->getSizeInCellsY());
    
    std::vector<int> grid(w * h);
    for(int i=0; i<w*h; ++i) {
        grid[i] = static_cast<int>(costmap->getCharMap()[i]);
    }

    // 2. Call your Core Library
    auto core_path = planner_core_->plan(
        grid, w, h, 
        {(int)start.pose.position.x, (int)start.pose.position.y}, 
        {(int)goal.pose.position.x, (int)goal.pose.position.y}
    );

    // 3. Translate to ROS msg
    for (auto& p : core_path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = global_frame_;
        ps.header.stamp = clock_->now();
        ps.pose.position.x = static_cast<double>(p.x);
        ps.pose.position.y = static_cast<double>(p.y);
        path.poses.push_back(ps);
    }
    return path;
}

} // namespace dynamic_obstacle_avoidance
