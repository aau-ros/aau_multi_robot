#include "robot_state_manager.h"

RobotStateManager::RobotStateManager(std::string node_name) {
    this->node_name = node_name;
    ros::NodeHandle nh;
    set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    try_to_lock_robot_state_sc = nh.serviceClient<robot_state::TryToLockRobotState>("robot_state/try_to_lock_robot_state");
    unlock_robot_state_sc = nh.serviceClient<robot_state::UnlockRobotState>("robot_state/unlock_robot_state");
}

unsigned int RobotStateManager::getRobotState() {
    robot_state::GetRobotState srv;
    while(!get_robot_state_sc.call(srv))
        ROS_ERROR("call to get_robot_state failed");
    return srv.response.robot_state;
}

void RobotStateManager::setRobotState(unsigned int robot_state) {
    ROS_INFO("setting state");
    robot_state::SetRobotState srv;
    srv.request.setting_node = node_name;
    srv.request.robot_state = robot_state;
    while(!set_robot_state_sc.call(srv))
        ROS_ERROR("call to get_robot_state failed");
}

void RobotStateManager::lockRobotState() {
    robot_state::TryToLockRobotState try_msg;
    try_msg.request.locking_node = node_name;
    bool repeat, call_succeeded;
    do {
        ROS_INFO("trying to acquire lock on robot_state");
        call_succeeded = try_to_lock_robot_state_sc.call(try_msg);
        if(!call_succeeded) {
            ROS_ERROR("failed call to try_lock");
            repeat = true;
        }
        if(!try_msg.response.lock_acquired) {
            ROS_INFO("lock not acquired: retrying");
            repeat = true;
        }
    } while(repeat);
}

void RobotStateManager::unlockRobotState() {
    robot_state::UnlockRobotState unlock_msg;
    while(!unlock_robot_state_sc.call(unlock_msg))
        ROS_ERROR("call to unlock_robot_state failed!");
}
