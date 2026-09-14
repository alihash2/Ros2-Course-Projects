#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace ms04_autonomous_navigation {

class CmdVelRelay : public rclcpp::Node {
public:
    CmdVelRelay() : Node("cmd_vel_relay") {
        // Declare parameter for custom topics if desired
        this->declare_parameter<std::string>("input_topic", "/cmd_vel_raw");
        this->declare_parameter<std::string>("output_topic", "/cmd_vel");
        this->declare_parameter<std::string>("frame_id", "base_footprint");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_topic, 10,
            std::bind(&CmdVelRelay::twist_callback, this, std::placeholders::_1));

        pub_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "CmdVelRelay running: '%s' (Twist) -> '%s' (TwistStamped, frame: '%s')",
                    input_topic.c_str(), output_topic.c_str(), frame_id_.c_str());
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = this->now();
        stamped.header.frame_id = frame_id_;
        stamped.twist = *msg;
        pub_stamped_->publish(stamped);
    }

    std::string frame_id_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
};

} // namespace ms04_autonomous_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ms04_autonomous_navigation::CmdVelRelay>());
    rclcpp::shutdown();
    return 0;
}
