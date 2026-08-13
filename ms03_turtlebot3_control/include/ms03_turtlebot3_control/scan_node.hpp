#include <memory>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanNode : public rclcpp::Node
{
    public:
        ScanNode();

    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

        double safety_distance_;
        double front_angle_deg_;

        bool is_path_blocked_;
        bool is_path_clear_;
};
