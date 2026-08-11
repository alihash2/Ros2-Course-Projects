#pragma once
#include <rclcpp/rclcpp.hpp>                // Core ROS 2 library: Node base class, timers, logging, and execution spinning
#include <geometry_msgs/msg/twist.hpp>      // Velocity message type: controls motion via linear.x and angular.z
#include <turtlesim/msg/pose.hpp>          // Robot feedback message type: provides live position (x, y) and orientation (theta)

class GoToPoseNode : public rclcpp::Node {
public:
    GoToPoseNode(float target_x, float target_y, float target_theta);

private:
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
    void controlLoop();
    double normalizeAngle(double angle);

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose current_pose_;
    bool pose_received_{false};

    double target_x_;
    double target_y_;
    double target_theta_;

    const double K_linear_{1.5};
    const double K_angular_{4.0};
    const double distance_tolerance_{0.05};
    const double angle_tolerance_{0.02};
};