#include "robot_state_manager.h"

RobotStateManager::RobotStateManager() {
    ros::NodeHandle nh;
    //get_robot_state_sc = nh;
}

RobotStateEM *RobotStateManager::getRobotState() {
    robot_state::GetRobotState get_srv_msg;
    bool call_succeeded = get_robot_state_sc.call(get_srv_msg);
    while(!call_succeeded) {
        ROS_ERROR("call failed");   
        call_succeeded = get_robot_state_sc.call(get_srv_msg);
    }
}
