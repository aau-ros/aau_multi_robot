#ifndef ROBOT_STATE_MANAGEMENT_H
#define ROBOT_STATE_MANAGEMENT_H

#include <unordered_map>
#include <ros/ros.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>
#include "robot_state.h"

class RobotStateApi
{
public:
    RobotStateApi();
//    RobotState *getRobotState();
//    bool getRobotState2();
//    void setRobotState(RobotState *state);
    RobotState *enumToState(unsigned int enum_val);
    unsigned int stateToEnum(RobotState *state);

private:
    std::unordered_map<unsigned int, RobotState*> enumToStateMap;
    std::unordered_map<RobotState*, unsigned int> stateToEnumMap;
//    ros::ServiceClient get_robot_state_sc;
//    ros::ServiceClient set_robot_state_sc;
//    ros::ServiceClient try_to_lock_robot_state_ss;
//    ros::ServiceClient unlock_robot_state_ss;

//    void createServiceClients();
    void fillMaps();
    void addStateToMaps(unsigned int enum_val, RobotState *state);
};

#endif // ROBOT_STATE_MANAGEMENT_H
