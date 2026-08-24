#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace dynamic_obstacle_avoidance {

class CmdVelRelay : public rclcpp::Node {
public:
    CmdVelRelay() : Node("cmd_vel_relay") {
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10,
            std::bind(&CmdVelRelay::twist_callback, this, std::placeholders::_1));

        pub_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "CmdVelRelay initialized: /cmd_vel_raw (Twist) -> /cmd_vel (TwistStamped)");
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = this->now();
        stamped.header.frame_id = "base_footprint";
        stamped.twist = *msg;
        pub_stamped_->publish(stamped);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
};

} // namespace dynamic_obstacle_avoidance

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_obstacle_avoidance::CmdVelRelay>());
    rclcpp::shutdown();
    return 0;
}
