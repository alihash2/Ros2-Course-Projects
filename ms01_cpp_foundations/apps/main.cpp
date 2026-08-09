#include "mobile_robot.hpp"
#include <iostream>
#include <iomanip>

int main() {
    MobileRobot robot("Trixie2");
    robot.setSpeed(2.0f, 2.0f);

    // [TASK 4 CHECKLIST ITEM: Log robot data in an easy readable format]
    std::cout << "========================================\n";
    std::cout << "       MOBILE ROBOT SENSOR LOGS         \n";
    std::cout << "========================================\n\n";

    std::cout << std::fixed << std::setprecision(2);

    for (int i = 1; i <= 3; ++i) {
        std::cout << "--- Reading #" << i << " ---\n";

        WheelData wheels = robot.getWheelData();
        std::cout << "[WHEELS]   Left: " << wheels.left_speed 
                  << " rad/s | Right: " << wheels.right_speed << " rad/s\n";

        OdometryData odom = robot.getOdometry();
        std::cout << "[ODOM]     X: " << odom.x 
                  << " m | Y: " << odom.y 
                  << " m | Heading: " << odom.z << " rad\n";

        LidarData lidar = robot.getLidardata();
        std::cout << "[LIDAR]    Ranges (m): [ ";
        for (float range : lidar.ranges) {
            std::cout << range << " ";
        }
        std::cout << "]\n\n";
    }

    robot.stop();
    return 0;
}