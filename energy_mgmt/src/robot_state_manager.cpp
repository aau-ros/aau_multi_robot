#include "robot_state_manager.h"
#include <stdlib.h>     /* srand, rand */ //TODO remove

RobotStateManager::RobotStateManager() {
    ;
}

RobotStateEM *RobotStateManager::getRobotState() {
    int v1 = rand() % 3;
    if(v1 == 0)
        return new RobotState1();
    else if(v1 == 1)
        return new RobotState2();
    else
        return new RobotState3();
}
