#pragma once
#include <string>
#include <vector>
#include <random>
#include "wheel_data.hpp"
#include "odometry.hpp"
#include "lidar_data.hpp"


class MobileRobot {
    private:
        std::string name;
        float left_speed;
        float right_speed;
        std::mt19937 rng; //random generator

    public:
        MobileRobot(std::string robot_name);

        void setSpeed(float left, float right);
        void printStatus();
        void stop();

        WheelData getWheelData();
        OdometryData getOdometry();
        LidarData getLidardata();
};