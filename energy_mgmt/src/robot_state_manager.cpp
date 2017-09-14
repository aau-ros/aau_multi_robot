#include "robot_state_manager.h"

RobotStateManager::RobotStateManager(std::string node_name) {
    if(node_name.empty())
        ROS_FATAL("EMPTY NODE NAME");
    this->node_name = node_name;
    counter = 0;
    
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
        ROS_ERROR("call to set_robot_state failed");
}

void RobotStateManager::lockRobotState() {
    robot_state::TryToLockRobotState try_msg;
    try_msg.request.locking_node = node_name;
    ros::Time start_time = ros::Time::now();
    bool repeat = false, call_succeeded = false;
    do {
        ROS_INFO("trying to acquire lock on robot_state");
        call_succeeded = try_to_lock_robot_state_sc.call(try_msg);
        if(!call_succeeded) {
            ROS_ERROR("failed call to try_lock");
            repeat = true;
            ros::Duration(1).sleep();
        }
        else {
            if(!try_msg.response.lock_acquired) {
                ROS_INFO("lock not acquired: retrying");
                repeat = true;
                ros::Duration(1).sleep();
                if(ros::Time::now() - start_time > ros::Duration(3*60)) {
                    ROS_FATAL("IMPOSSIBLE TO ACQUIRE LOCK");
                    ros::Duration(59).sleep();
                }
            } else
                repeat = false;
        }
    } while(repeat);
    counter = try_msg.response.counter;
    ROS_INFO("lock acquired; counter: %u", counter);
}

void RobotStateManager::unlockRobotState() {
    robot_state::UnlockRobotState unlock_msg;
    unlock_msg.response.lock_released = false;
    bool lock_released = false;
    while(!lock_released) {
        while(!unlock_robot_state_sc.call(unlock_msg))
            ROS_ERROR("call to unlock_robot_state failed!");
        lock_released = unlock_msg.response.lock_released;
        if(!lock_released)
            ROS_ERROR("unlock_robot_state didn't release the lock");
    }
    if(counter != unlock_msg.response.counter)
        ROS_ERROR("counter != unlock_msg.response.counter: %u != %u", counter, unlock_msg.response.counter);
    ROS_INFO("lock released; counter: %u", counter);
}
