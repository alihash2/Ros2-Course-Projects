#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dynamic_obstacle_avoidance {

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower_node") {
        // ROS Parameters for easy tuning
        this->declare_parameter("kp_angular", 2.0);
        this->declare_parameter("ki_angular", 0.0);
        this->declare_parameter("kd_angular", 0.1);
        this->declare_parameter("lqr_k1", 1.5); // Position error gain
        this->declare_parameter("lqr_k2", 0.5); // Velocity error gain
        this->declare_parameter("target_linear_velocity", 0.22); // TB3 max is ~0.22

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollowerNode::path_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), std::bind(&PathFollowerNode::odom_callback, this, std::placeholders::_1));

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Control loop timer (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PathFollowerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Path Follower Node Initialized (PID Heading + LQR Velocity)");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg;
        current_waypoint_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses.", path_->poses.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_twist_ = msg->twist.twist;
        has_odom_ = true;
    }

    void control_loop() {
        if (!has_odom_ || !path_ || path_->poses.empty()) {
            return;
        }

        // Check if we reached the end of the path
        if (current_waypoint_idx_ >= path_->poses.size()) {
            publish_zero_velocity();
            return;
        }

        // Get target waypoint coordinate
        auto target_pose = path_->poses[current_waypoint_idx_].pose;
        double tx = target_pose.position.x;
        double ty = target_pose.position.y;

        // Get current robot state
        double rx = current_pose_.position.x;
        double ry = current_pose_.position.y;
        
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, r_theta;
        m.getRPY(roll, pitch, r_theta);

        // Calculate distance and angle to target
        double dx = tx - rx;
        double dy = ty - ry;
        double distance_to_target = std::sqrt(dx*dx + dy*dy);

        // If close to waypoint, switch to next waypoint
        const double waypoint_tolerance = 0.3; // meters
        if (distance_to_target < waypoint_tolerance) {
            current_waypoint_idx_++;
            RCLCPP_INFO(this->get_logger(), "Switching to waypoint index: %zu", current_waypoint_idx_);
            return;
        }

        // 1. Heading Controller: PID
        double target_theta = std::atan2(dy, dx);
        double angle_error = target_theta - r_theta;

        // Normalize angle to [-M_PI, M_PI]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        double kp_a = this->get_parameter("kp_angular").as_double();
        double ki_a = this->get_parameter("ki_angular").as_double();
        double kd_a = this->get_parameter("kd_angular").as_double();

        // PID Math
        double dt = 0.05; // 20Hz
        angular_integral_ += angle_error * dt;
        double angular_derivative = (angle_error - last_angle_error_) / dt;
        last_angle_error_ = angle_error;

        double angular_vel = (kp_a * angle_error) + (ki_a * angular_integral_) + (kd_a * angular_derivative);

        // 2. Velocity Controller: LQR Follower
        // State error: x_err = [position_error, velocity_error]^T
        double target_v = this->get_parameter("target_linear_velocity").as_double();
        double current_v = current_twist_.linear.x;

        double pos_error = distance_to_target;
        double vel_error = target_v - current_v;

        double k1 = this->get_parameter("lqr_k1").as_double();
        double k2 = this->get_parameter("lqr_k2").as_double();

        // LQR Feedback: u = -K * x -> v_cmd = target_v + (k1 * pos_error + k2 * vel_error)
        // We cap the maximum speed to prevent erratic jumps.
        double linear_vel = target_v + (k1 * pos_error + k2 * vel_error);
        linear_vel = std::max(0.0, std::min(linear_vel, target_v));

        // If angle error is huge, slow down linear velocity to turn on spot first
        if (std::abs(angle_error) > 0.5) {
            linear_vel *= 0.1;
        }

        // Publish velocities
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd_vel);
    }

    void publish_zero_velocity() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    // Subs, Pubs, and Timers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Path Tracking State
    nav_msgs::msg::Path::SharedPtr path_;
    size_t current_waypoint_idx_ = 0;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_twist_;
    bool has_odom_ = false;

    // PID Memory
    double last_angle_error_ = 0.0;
    double angular_integral_ = 0.0;
};

} // namespace dynamic_obstacle_avoidance

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dynamic_obstacle_avoidance::PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
