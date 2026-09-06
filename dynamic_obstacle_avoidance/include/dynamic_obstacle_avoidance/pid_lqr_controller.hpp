#ifndef DYNAMIC_OBSTACLE_AVOIDANCE__PID_LQR_CONTROLLER_HPP_
#define DYNAMIC_OBSTACLE_AVOIDANCE__PID_LQR_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace dynamic_obstacle_avoidance {

class PidLqrController : public nav2_core::Controller {
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                   std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;
    std::string global_frame_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // Plan tracking
    nav_msgs::msg::Path global_plan_;
    size_t current_waypoint_idx_ = 0;
    bool plan_initialized_ = false;

    // Tuning parameters
    double kp_angular_ = 2.0;
    double ki_angular_ = 0.0;
    double kd_angular_ = 0.1;
    double lqr_k1_ = 1.5;
    double lqr_k2_ = 0.5;
    double target_linear_velocity_ = 0.22;
    double speed_limit_ = std::numeric_limits<double>::max();
    bool use_speed_limit_ = false;

    // Final approach / goal alignment. The nav2 goal checker requires both
    // xy <= xy_goal_tolerance (0.25 m) and yaw <= yaw_goal_tolerance (0.25 rad)
    // vs. the goal pose, so the controller must drive past its own stop radius
    // and rotate to the goal orientation before returning zero velocity.
    double final_approach_tolerance_ = 0.15;
    double yaw_align_tolerance_ = 0.08;
    double kp_final_yaw_ = 2.5;

    const double waypoint_tolerance_ = 0.3;
    const double max_angular_vel_ = 1.8;
    const double control_period_ = 0.05;  // 20 Hz

    // PID memory
    double last_angle_error_ = 0.0;
    double angular_integral_ = 0.0;

    rclcpp::Clock::SharedPtr clock_;

    double normalize_angle(double angle) const;
};

}  // namespace dynamic_obstacle_avoidance
#endif  // DYNAMIC_OBSTACLE_AVOIDANCE__PID_LQR_CONTROLLER_HPP_
