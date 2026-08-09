#include <random>
#include "lidar_data.hpp"
#include "mobile_robot.hpp"

LidarData MobileRobot::getLidardata(){
    std::uniform_real_distribution<float> range_dist(0.2f, 10.0f);

    LidarData data;
    for (int i = 0; i<5; ++i){
        float random_distance = range_dist(rng);

        data.ranges.push_back(random_distance);
    }

    return data;
}
