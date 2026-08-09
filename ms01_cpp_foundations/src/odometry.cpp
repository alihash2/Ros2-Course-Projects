#include <random>
#include "odometry.hpp"
#include "mobile_robot.hpp"

OdometryData MobileRobot::getOdometry(){
    std::uniform_real_distribution<float> pos_dist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> angle_dist(-3.14f, 3.14f);

    OdometryData data;
    data.x = pos_dist(rng);
    data.y = pos_dist(rng);
    data.z = angle_dist(rng);

    return data;
}