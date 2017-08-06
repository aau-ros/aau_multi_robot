#include <robot_state_manager.h>

RobotStateManager::RobotStateManager()
{
    ROS_INFO("Creating instance of RobotStateManager class...");
    
    ROS_INFO("Instance correctly created");
}

void RobotStateManager::createServices() {
    ros::NodeHandle nh;
    ss_get_robot_state = nh.advertiseService("robot_state/get_robot_state", &RobotStateManager::callback_get_robot_state, this);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool RobotStateManager::callback_get_robot_state(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res) {
    ROS_INFO("get_robot_state service required");
    res.robot_state = robot_state;
    ROS_INFO("Service message successfully sent");
    return true;
}
#pragma GCC diagnostic pop
