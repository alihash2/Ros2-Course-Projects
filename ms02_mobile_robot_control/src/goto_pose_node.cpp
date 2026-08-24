#include "ms02_mobile_robot_control/goto_pose_node.hpp"
#include <cmath>
#include <functional>
#include <chrono>

// Constructor: Initialize ROS 2 Node, subscriber, publisher, and control timer
GoToPoseNode::GoToPoseNode(float target_x, float target_y, float target_theta)
: Node("goto_pose_node"),
  target_x_(target_x),
  target_y_(target_y),
  target_theta_(target_theta)
{
    // 1. Subscribe to live turtle pose feedback (Queue size: 10)
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&GoToPoseNode::poseCallback, this, std::placeholders::_1)
    );

    // 2. Publisher for robot movement commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);

    // 3. Control loop timer executing at ~30 Hz (33 ms interval)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&GoToPoseNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
        "GoToPoseNode initialized with Target: (x=%.2f, y=%.2f, theta=%.2f rad)",
        target_x_, target_y_, target_theta_);
}

// Callback: Updates internal pose feedback state
void GoToPoseNode::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    current_pose_ = *msg;
    pose_received_ = true;
}

// Assign a new target pose at runtime and re-arm the goal-reached flag
void GoToPoseNode::setTarget(double target_x, double target_y, double target_theta)
{
    target_x_ = target_x;
    target_y_ = target_y;
    target_theta_ = target_theta;
    has_target_ = true;
    goal_reached_ = false;

    RCLCPP_INFO(this->get_logger(),
        "New target set: (x=%.2f, y=%.2f, theta=%.2f rad)",
        target_x_, target_y_, target_theta_);
}

// Helper: Keeps heading errors strictly within [-pi, +pi] radians
double GoToPoseNode::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Core 30 Hz Proportional Control Loop
void GoToPoseNode::controlLoop()
{
    // Guard check: wait until a first pose arrives AND the app has assigned
    // a real target. Without this the constructor's placeholder (0,0,0)
    // would make the turtle drive off (and log "reached") at launch.
    if (!pose_received_ || !has_target_) {
        return;
    }

    // Compute translational coordinate errors
    double dx = target_x_ - current_pose_.x;
    double dy = target_y_ - current_pose_.y;
    double distance_error = std::hypot(dx, dy); // sqrt(dx^2 + dy^2)

    geometry_msgs::msg::Twist cmd_vel;

    // Stage A: Drive to Target (x, y)
    if (distance_error > distance_tolerance_) {
        double desired_heading = std::atan2(dy, dx);
        double heading_error = normalizeAngle(desired_heading - current_pose_.theta);

        // Proportional controller law
        cmd_vel.linear.x = K_linear_ * distance_error;
        cmd_vel.angular.z = K_angular_ * heading_error;
    }
    // Stage B: Align Final Orientation (theta)
    else {
        double final_angle_error = normalizeAngle(target_theta_ - current_pose_.theta);

        if (std::abs(final_angle_error) > angle_tolerance_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = K_angular_ * final_angle_error;
        } else {
            // Reached target pose within tolerances
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);

            // Announce arrival exactly once per target — the timer keeps
            // ticking, so without this guard the log would repeat forever.
            if (!goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Target pose successfully reached! (%.2f, %.2f, theta=%.2f)",
                    target_x_, target_y_, target_theta_);
            }
            return;
        }
    }

    // Publish calculated velocity commands to robot
    cmd_vel_pub_->publish(cmd_vel);
}