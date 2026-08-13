#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class MotionNode : public rclcpp::Node{
    public:
        MotionNode();
    
    private:
        void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
        void GoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

        double x_{0}, y_{0}, yaw_{0};
        double gx_{0}, gy_{0};
        bool active_{false};
};