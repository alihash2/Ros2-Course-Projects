#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <limits>
#include <thread>
#include <chrono>
#include "ms02_mobile_robot_control/goto_pose_node.hpp"

// Turtlesim window is 11.08 units wide/high, but the usable grid the user
// asked to enforce is 0..10 inclusive on both X and Y.
constexpr double kMinLimit = 0.0;
constexpr double kMaxLimit = 10.0;

// Reads a float from stdin, re-prompting until a valid number inside
// [min, max] is entered.
double promptForValue(const std::string& name, double min, double max)
{
    while (true) {
        std::cout << "Enter " << name << " (range " << min << " to " << max << "): ";
        std::string line;
        if (!std::getline(std::cin, line)) {
            // EOF / Ctrl+D: fall back to min so we never loop forever
            return min;
        }
        try {
            size_t idx = 0;
            double value = std::stod(line, &idx);
            // Reject trailing garbage like "3abc"
            if (idx != line.size()) throw std::invalid_argument("trailing chars");
            if (value < min || value > max) {
                std::cout << "  [!] " << name << " must be between "
                          << min << " and " << max << ". Try again.\n";
                continue;
            }
            return value;
        } catch (const std::exception&) {
            std::cout << "  [!] Invalid number '" << line << "'. Try again.\n";
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GoToPoseNode>(0.0, 0.0, 0.0);

    // Spin the node in a background thread so the menu stays responsive
    // while the control loop runs.
    std::thread spin_thread([node]() { rclcpp::spin(node); });

    std::cout << "\n=============================================\n";
    std::cout << "   TURTLESIM GO-TO-POSE INTERACTIVE MENU\n";
    std::cout << "=============================================\n";
    std::cout << "Valid pose limits -> X: 0..10 | Y: 0..10\n";
    std::cout << "Theta is any angle in radians (it will be normalized).\n";

    bool running = true;
    while (running && rclcpp::ok()) {
        std::cout << "\n---------------------------------------------\n";
        std::cout << " 1. Send turtle to a pose\n";
        std::cout << " 2. Exit\n";
        std::cout << "---------------------------------------------\n";

        std::cout << "Select option [1-2]: ";
        std::string choice;
        if (!std::getline(std::cin, choice)) break;

        if (choice == "1") {
            std::cout << "\n--- Enter Target Pose ---\n";
            double x = promptForValue("X", kMinLimit, kMaxLimit);
            double y = promptForValue("Y", kMinLimit, kMaxLimit);
            double theta = promptForValue("Theta (radians)", -6.28318530718, 6.28318530718);

            node->setTarget(x, y, theta);

            // Wait until the control loop reports arrival
            std::cout << "Navigating to (" << x << ", " << y << ", theta=" << theta << ")...\n";
            while (rclcpp::ok() && !node->isGoalReached()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (rclcpp::ok()) {
                std::cout << "[SUCCESS] Turtle reached the target pose!\n";
            }
        } else if (choice == "2") {
            std::cout << "\n[INFO] Exiting. Goodbye!\n";
            running = false;
        } else {
            std::cout << "[!] Invalid selection. Enter 1 or 2.\n";
        }
    }

    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 0;
}
