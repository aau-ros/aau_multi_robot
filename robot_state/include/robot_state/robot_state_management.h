#ifndef ROBOT_STATE_MANAGEMENT_H
#define ROBOT_STATE_MANAGEMENT_H

#include <unordered_map>
#include <ros/ros.h>
#include <robot_state.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>

class RobotStateApi
{
public:
    RobotStateApi();
    RobotState *getRobotState();

private:
    std::unordered_map<unsigned int, RobotState*> stateMap;
    ros::ServiceClient get_robot_state_sc;
    ros::ServiceClient set_robot_state_sc;

    void createServiceClients();
    void createRobotStateInstances();
};

#endif // ROBOT_STATE_MANAGEMENT_H
