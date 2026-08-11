#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include "ms02_mobile_robot_control/goto_pose_node.hpp"

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    float target_x = 7.0f;
    float target_y = 7.0f;
    float target_theta = 0.0f;

    if (argc >= 4){
        target_x = static_cast<float>(std::atof(argv[1]));
        target_y = static_cast<float>(std::atof(argv[2]));
        target_theta = static_cast<float>(std::atof(argv[3]));
    }

    auto node = std::make_shared<GoToPoseNode>(target_x, target_y, target_theta);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}