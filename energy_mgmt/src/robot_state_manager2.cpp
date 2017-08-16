#include "robot_state_manager2.h"

RobotStateManager2::RobotStateManager2() {}

void RobotStateManager2::setRobotState(unsigned int robot_state) {
    this->robot_state = robot_state;
}

unsigned int RobotStateManager2::getRobotState() {
    return robot_state;
}

void RobotStateManager2::lockRobotState() {} 

void RobotStateManager2::unlockRobotState() {}
