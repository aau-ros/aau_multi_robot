#ifndef ROBOT_STATE_MANAGER2_H
#define ROBOT_STATE_MANAGER2_H

#include "robot_state_manager_interface.h"

class RobotStateManager2 : public RobotStateManagerInterface
{
public:
    RobotStateManager2();
    unsigned int getRobotState();
    void setRobotState(unsigned int robot_state);
    void lockRobotState();
    void unlockRobotState();

private:
    unsigned int robot_state;
};


#endif // ROBOT_STATE_MANAGER2_H
