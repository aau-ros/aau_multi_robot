#include "robot_state/robot_state_management.h"

RobotStateApi::RobotStateApi() {
    ROS_INFO("Creating instance of RobotStateApi");
    createServiceClients();
    createRobotStateInstances();
    ROS_INFO("Instance correctly created");
}

void RobotStateApi::createServiceClients() {
    ros::NodeHandle nh;
    set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    try_to_lock_robot_state_ss = nh.serviceClient<robot_state::SetRobotState>("robot_state/try_to_lock_robot_state");
    unlock_robot_state_ss = nh.serviceClient<robot_state::SetRobotState>("robot_state/unlock_robot_state");
}

void RobotStateApi::createRobotStateInstances() { //TODO complete stateMap
    stateMap.insert({robot_state::INITIALIZING, new InitializingState()});
}

RobotState *RobotStateApi::getRobotState() { //TODO raise exception
    robot_state::GetRobotState get_srv_msg;
    bool call_succeeded = get_robot_state_sc.call(get_srv_msg);
    while(!call_succeeded) {
        ROS_ERROR("call failed");   
        call_succeeded = get_robot_state_sc.call(get_srv_msg);
    }
    return stateMap.at(get_srv_msg.response.robot_state); //TODO safety check to avoid out_of_range
}
