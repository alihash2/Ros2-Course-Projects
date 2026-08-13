#include "ms03_turtlebot3_control/odom_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

OdomNode::OdomNode() : Node("odom_node"){
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&OdomNode::odom_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Odom Node started. Listening to /odom...");
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

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Pose -> x: %.2f, y: %.2f, yaw: %.2f rad",
        current_x_, current_y_, current_yaw_
    );
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}