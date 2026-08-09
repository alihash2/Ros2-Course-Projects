#include "mobile_robot.hpp"
#include <iostream>

MobileRobot::MobileRobot(std::string robot_name) : rng(std::random_device{}()){
    name = robot_name;
    left_speed = 0.0f;
    right_speed = 0.0f;
}

void MobileRobot::setSpeed(float left, float right){
    left_speed = left;
    right_speed = right;
}

void MobileRobot::printStatus(){
    std::cout << "[" << name << "] Left Speed: " << left_speed << " | Right Speed: " << right_speed << std::endl;
}

void MobileRobot::stop(){
    left_speed = 0.0f;
    right_speed = 0.0f;
    std::cout << "[" << name << "] Stopped." << std::endl; 
}
