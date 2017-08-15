#include "robot_state_manager2.h"

RobotStateManager2::RobotStateManager2() {
    robot_state = robot_state::INITIALIZING;
}

void RobotStateManager2::setRobotState(unsigned int robot_state) {
    this->robot_state = robot_state;
}

unsigned int RobotStateManager2::getRobotState() {
    return robot_state;
}
