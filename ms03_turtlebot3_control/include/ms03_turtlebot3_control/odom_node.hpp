#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdomNode : public rclcpp::Node{
    public:
        OdomNode();
    
    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        double current_x_{0.0};
        double current_y_{0.0};
        double current_yaw_{0.0};
};