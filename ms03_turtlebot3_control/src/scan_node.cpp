#include "ms03_turtlebot3_control/scan_node.hpp"

ScanNode::ScanNode()
: Node("scan_node"),
is_path_blocked_(false),
is_path_clear_(true)
{
    this->declare_parameter<double>("safety_distance", 0.38);
    this->declare_parameter<double>("front_angle_deg", 25.0);

    this->get_parameter("safety_distance", safety_distance_);
    this->get_parameter("front_angle_deg", front_angle_deg_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(10).best_effort(),
        std::bind(&ScanNode::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ScanNode initialized. Safety Threshold: %.2f", safety_distance_);

}

void ScanNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(msg->ranges.empty()){
        return;
    }

    double min_front_dist = std::numeric_limits<double>::infinity();
    double min_left_dist = std::numeric_limits<double>::infinity();
    double min_right_dist = std::numeric_limits<double>::infinity();

    size_t total_beams = msg->ranges.size();

    for(size_t i = 0; i < total_beams; ++i){
        float range = msg->ranges[i];

        if(std::isnan(range) || std::isinf(range))
        {
            continue;
        }

        if (range > 0.0f && range < msg->range_min) {
            range = msg->range_min;
        }

        if (range <= 0.0f || range > msg->range_max) {
            continue;
        }

        double angle_rad = msg->angle_min + i * msg->angle_increment;
        angle_rad = std::atan2(std::sin(angle_rad), std::cos(angle_rad));
        double angle_deg = angle_rad * (180.0 / M_PI);

        if(std::abs(angle_deg) <= front_angle_deg_){
            if(range < min_front_dist) min_front_dist = range;
        }
        else if (angle_deg >= 20.0 && angle_deg <= 85.0){
            if(range < min_left_dist) min_left_dist = range;
        }
        else if (angle_deg <= -20.0 && angle_deg >= -85.0){
            if (range < min_right_dist) min_right_dist = range;
        }
    }

    if (min_front_dist < safety_distance_ || min_left_dist < 0.28 || min_right_dist < 0.28){
        is_path_blocked_ = true;
        is_path_clear_ = false;
    }
    else{
        is_path_blocked_ = false;
        is_path_clear_ = true;
    }

    std::string front_str = (min_front_dist == std::numeric_limits<double>::infinity()) ? "CLEAR" : std::to_string(min_front_dist).substr(0,4) + "m";
    std::string left_str = (min_left_dist == std::numeric_limits<double>::infinity()) ? "CLEAR" : std::to_string(min_left_dist).substr(0,4) + "m";
    std::string right_str = (min_right_dist == std::numeric_limits<double>::infinity()) ? "CLEAR" : std::to_string(min_right_dist).substr(0,4) + "m";

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, 
    "[SCAN NODE] Front: [%s] | Left: [%s] | Right: [%s] | Path: %s",
    front_str.c_str(), left_str.c_str(), right_str.c_str(),
    is_path_blocked_ ? "BLOCKED!" : "CLEAR"
    );

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

