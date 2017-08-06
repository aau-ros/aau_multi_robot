#include <robot_state_manager.h>

//TODO use a separate executable???
RobotStateManager::RobotStateManager()
{
    ROS_INFO_COND_NAMED(LOG_TRUE, "robot_state", "Creating instance of RobotStateManager class..."); //TODO use COND_NAMED everywhere
    initializeRobotState();
    createServices();    
    fillRobotStateStringsVector();
    ROS_INFO("Instance correctly created");
}

void RobotStateManager::initializeRobotState() {
    robot_state = robot_state::INITIALIZING;
}

void RobotStateManager::createServices() {
    ros::NodeHandle nh;
    ss_get_robot_state = nh.advertiseService("robot_state/get_robot_state", &RobotStateManager::get_robot_state_callback, this);
    ss_set_robot_state = nh.advertiseService("robot_state/set_robot_state", &RobotStateManager::set_robot_state_callback, this);
    //TODO use lock and unlock???
    //ss_lock_robot_state = nh.advertiseService("robot_state/lock_robot_state", &RobotStateManager::lock_robot_state_callback, this);
    //ss_unlock_robot_state = nh.advertiseService("robot_state/unlock_robot_state", &RobotStateManager::unlock_robot_state_callback, this);
}

//TODO not very robust to changes... try using enumerations or other ways...
void RobotStateManager::fillRobotStateStringsVector() {
    std::vector<std::string>::iterator it = robot_state_strings.begin();
    robot_state_strings.push_back("INITIALIZING");
    robot_state_strings.push_back("COMPUTING");
    robot_state_strings.push_back("EXPLORING");
    robot_state_strings.push_back("GOING_CHECKING_VACANCY");
    robot_state_strings.push_back("GOING_CHARGING");
    robot_state_strings.push_back("CHARGING");
    robot_state_strings.push_back("FULLY_CHARGED");
    robot_state_strings.push_back("LEAVING_DS");
    robot_state_strings.push_back("GOING_IN_QUEUE");
    robot_state_strings.push_back("IN_QUEUE");
    robot_state_strings.push_back("AUCTIONING");
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool RobotStateManager::get_robot_state_callback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res) {
    mutex.lock();
    ROS_INFO("get_robot_state service required");    
    res.robot_state = robot_state;
    ROS_INFO("Service message successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool RobotStateManager::set_robot_state_callback(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res) {
    mutex.lock();
    ROS_INFO("set_robot_state service required");
    //TODO sanity check
    ROS_DEBUG("Robot state transiction: %s -> %s", robotStateEnumToString(robot_state).c_str(), robotStateEnumToString(req.robot_state).c_str()); //TODO
    robot_state = req.robot_state;     
    ROS_INFO("Service message successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool RobotStateManager::lock_robot_state_callback(robot_state::LockRobotState::Request &req, robot_state::LockRobotState::Response &res) {
    mutex.lock();
    ROS_INFO("lock_robot_state service required");    
    ROS_INFO("Service message successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool RobotStateManager::unlock_robot_state_callback(robot_state::UnlockRobotState::Request &req, robot_state::UnlockRobotState::Response &res) {
    mutex.lock();
    ROS_INFO("unlock_robot_state service required");
    ROS_INFO("Service message successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

std::string RobotStateManager::robotStateEnumToString(unsigned int enum_value) {
    // Sanity check
    if(enum_value >= robot_state_strings.size())
        ROS_FATAL("Invalid argument! It will cause a segmentation fault!");
    return robot_state_strings.at(enum_value);
}

