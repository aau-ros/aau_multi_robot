#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <unordered_map>

#include <ros/ros.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>
#include <robot_state/TryToLockRobotState.h>
#include <robot_state/UnlockRobotState.h>
#include "robot_state_manager_interface.h"

class RobotStateManager : public RobotStateManagerInterface
{
public:
    RobotStateManager(std::string node_name);
    unsigned int getRobotState();
    void setRobotState(unsigned int robot_state);
    void lockRobotState();
    void unlockRobotState();

private:
    std::string node_name;
    unsigned int counter;
    ros::ServiceClient get_robot_state_sc;
    ros::ServiceClient set_robot_state_sc;
    ros::ServiceClient try_to_lock_robot_state_sc;
    ros::ServiceClient unlock_robot_state_sc;
};


#endif // ROBOT_STATE_MANAGER_H
