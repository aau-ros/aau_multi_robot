#include "robot_state_manager.h"

RobotStateManager::RobotStateManager() {
    ros::NodeHandle nh;
    set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");

    stateMap.insert({robot_state::INITIALIZING, new InitializingState()});
}

RobotStateEM *RobotStateManager::getRobotState() { //TODO raise exception
    robot_state::GetRobotState get_srv_msg;
    bool call_succeeded = get_robot_state_sc.call(get_srv_msg);
    while(!call_succeeded) {
        ROS_ERROR("call failed");   
        call_succeeded = get_robot_state_sc.call(get_srv_msg);
    }
    return stateMap.at(get_srv_msg.response.robot_state); //TODO safety check to avoid out_of_range
}
