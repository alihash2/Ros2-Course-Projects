#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class GoToPoseNode : public rclcpp::Node{
    public:
        GoToPoseNode(float target_x, float target_y, float target_theta);

    private:
        void poseCallback(const turtlesim::msg::pose::SharedPtr msg);
        void controlLoop();
        double normalizeAngle(double angle);

        rclcpp::Subscription<turtlesim::msg::pose>::SharedPtr pose_sub_;
        rclcpp::Publisher<geometry_msgs/msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        turtlesim::msg::pose current_pose_;
        bool pose_recieved_{false};

        double target_x_;
        double target_y_;
        double target_thetha_;

        const double K_linear_{1.5};
        const double K_angular_{4.0};
        const double distance_tolerance_{0.05};
        const double angle_tolerance_{0.02};
};