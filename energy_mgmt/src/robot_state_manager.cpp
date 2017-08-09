#include "robot_state_manager.h"

RobotStateManager::RobotStateManager() {
    ros::NodeHandle nh;
    //get_robot_state_sc = nh;
}

RobotStateEM *RobotStateManager::getRobotState() { //TODO raise exception
    robot_state::GetRobotState get_srv_msg;
    bool call_succeeded = get_robot_state_sc.call(get_srv_msg);
    while(!call_succeeded) {
        ROS_ERROR("call failed");   
        call_succeeded = get_robot_state_sc.call(get_srv_msg);
    }

    if(get_srv_msg.response.robot_state == robot_state::INITIALIZING)
        return new InitializingState();
    else if(get_srv_msg.response.robot_state == robot_state::AUCTIONING)
        return NULL;
    else
        ROS_FATAL("INVALID");
}
