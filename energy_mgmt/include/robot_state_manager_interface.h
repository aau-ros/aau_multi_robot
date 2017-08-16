#ifndef ROBOT_STATE_MANAGER_INTERFACE_H
#define ROBOT_STATE_MANAGER_INTERFACE_H

#include <unordered_map>

#include <ros/ros.h>

class RobotStateManagerInterface
{
public:
    virtual unsigned int getRobotState() = 0;
    virtual void setRobotState(unsigned int robot_state) = 0;
    virtual void lockRobotState() = 0;
    virtual void unlockRobotState() = 0;
};

#endif // ROBOT_STATE_MANAGER_INTERFACE_H
