#include "ms03_turtlebot3_control/odom_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Odometry frame offset — /odom is relative to the robot's spawn point, so
// world goals from /user_goal must be shifted before comparing with /odom.
// Must match the constants in bt_executor_node.cpp.
constexpr double SPAWN_X = -2.0;
constexpr double SPAWN_Y = -0.5;

OdomNode::OdomNode() : Node("odom_node"){
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&OdomNode::odom_callback, this, std::placeholders::_1)
    );
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

    // Track the live goal so we can log remaining euclidean distance
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/user_goal", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            // /user_goal is in WORLD coordinates — convert to odom frame
            goal_x_ = msg->pose.position.x - SPAWN_X;
            goal_y_ = msg->pose.position.y - SPAWN_Y;
            has_goal_ = true;
        });

    RCLCPP_INFO(this->get_logger(), "Odom Node started. Listening to /odom and /user_goal, publishing to /current_pose...");
}

void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;
    pose_pub_->publish(pose_msg);

    // Live distance-to-goal log (euclidean). Only once a goal has actually
    // been received on /user_goal — no pointless spam while idle at spawn.
    if (has_goal_) {
        double dist = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[GOAL TRACKER] Goal (world): (%.2f, %.2f) | Distance remaining: %.2f m",
            goal_x_ + SPAWN_X, goal_y_ + SPAWN_Y, dist
        );
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}
