#include "ms03_turtlebot3_control/motion_node.hpp"

MotionNode::MotionNode() : Node("motion_node"){
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(), std::bind(&MotionNode::OdomCb, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&MotionNode::GoalCb, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "MotionNode initialized. Waiting for /goal_pose...");
}

void MotionNode::GoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    gx_ = msg->pose.position.x;
    gy_ = msg->pose.position.y;
    active_ = true;
    RCLCPP_INFO(this->get_logger(), "New target recieved: x=%.2f, y=%.2f", gx_, gy_);
}

void MotionNode::OdomCb(nav_msgs::msg::Odometry::SharedPtr msg){
    if (!active_) return;

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p;
    tf2::Matrix3x3(q).getRPY(r, p, yaw_);

    double dx = gx_ - x_;
    double dy = gy_ - y_;
    double dist = std::sqrt(dx*dx + dy*dy);
    double heading_err = std::atan2(dy, dx) - yaw_;

    while (heading_err > M_PI) heading_err -= 2.0 * M_PI;
    while (heading_err <-M_PI) heading_err += 2.0 * M_PI;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.header.frame_id = "base_link";

    if(dist < 0.05){
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Goal Reached!");
        active_ = false;
        return;
    }
    

    double v = std::clamp(0.5 * dist, -0.22, 0.22);
    double w = std::clamp(1.5 * heading_err, -1.0, 1.0);
    
    if (std::abs(heading_err) > 0.5){
        v *= 0.2;
    }

    cmd.twist.linear.x = v;
    cmd.twist.angular.z = w;
    cmd_pub_->publish(cmd);

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionNode>());
    rclcpp::shutdown();
    return 0;
}