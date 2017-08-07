#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <ros/ros.h>
#include <robot_state/robot_state.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>
#include "robot_state_em.h"

class RobotStateManager
{
public:
    RobotStateManager();
    RobotStateEM *getRobotState();

private:
    ros::ServiceClient get_robot_state_sc;
};


#endif // ROBOT_STATE_MANAGER_H
