#include <random>
#include "wheel_data.hpp"
#include "mobile_robot.hpp"

WheelData MobileRobot::getWheelData(){
    std::uniform_real_distribution<float> noise(-0.1f, 0.1f);
    WheelData data;
    data.left_speed = left_speed + noise(rng);
    data.right_speed = right_speed + noise(rng);
    return data;
}