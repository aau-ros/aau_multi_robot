#ifndef ROBOT_STATE_MANAGER2_H
#define ROBOT_STATE_MANAGER2_H

#include <unordered_map>

#include <ros/ros.h>
#include <robot_state/robot_state_management.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>

class RobotStateManager2
{
public:
    RobotStateManager2();
    unsigned int getRobotState();
    void setRobotState(unsigned int robot_state);

protected:
    unsigned int robot_state;
};


#endif // ROBOT_STATE_MANAGER2_H
