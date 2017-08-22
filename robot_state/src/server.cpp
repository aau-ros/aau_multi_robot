#include <server.h>

//TODO use a separate executable???
Server::Server()
{
    ROS_INFO_COND_NAMED(LOG_TRUE, "robot_state", "Creating instance of Server class..."); //TODO use COND_NAMED everywhere
    initializeRobotState();
    createServices();
    fillRobotStateStringsVector();
    createLogFile();
    ROS_INFO("Instance correctly created");
}

void Server::initializeRobotState() {
    robot_state = robot_state::INITIALIZING;
    state_locked = false;
    counter = 0;
    locking_node = "";
}

void Server::createServices() {
    ros::NodeHandle nh;
    get_robot_state_ss = nh.advertiseService("robot_state/get_robot_state", &Server::getRobotStateCallback, this);
    set_robot_state_ss = nh.advertiseService("robot_state/set_robot_state", &Server::setRobotStateCallback, this);
    try_to_lock_robot_state_ss = nh.advertiseService("robot_state/try_to_lock_robot_state", &Server::tryToLockRobotStateCallback, this);
    unlock_robot_state_ss = nh.advertiseService("robot_state/unlock_robot_state", &Server::unlockRobotStateCallback, this);
}

//TODO not very robust to changes... try using enumerations or other ways...
void Server::fillRobotStateStringsVector() {
    std::vector<std::string>::iterator it = robot_state_strings.begin();
    robot_state_strings.push_back("INITIALIZING");
    robot_state_strings.push_back("CHOOSING_ACTION");
    robot_state_strings.push_back("COMPUTING_NEXT_GOAL");
    robot_state_strings.push_back("MOVING_TO_FRONTIER");
    robot_state_strings.push_back("GOING_CHECKING_VACANCY");
    robot_state_strings.push_back("CHECKING_VACANCY");
    robot_state_strings.push_back("GOING_CHARGING");
    robot_state_strings.push_back("CHARGING");
    robot_state_strings.push_back("CHARGING_COMPLETED");
    robot_state_strings.push_back("CHARGING_ABORTED");
    robot_state_strings.push_back("LEAVING_DS");
    robot_state_strings.push_back("GOING_IN_QUEUE");
    robot_state_strings.push_back("IN_QUEUE");
    robot_state_strings.push_back("AUCTIONING");
    robot_state_strings.push_back("auctioning_2");
    robot_state_strings.push_back("exploring_for_graph_navigation");
    robot_state_strings.push_back("stopped");
    robot_state_strings.push_back("stuck");
    robot_state_strings.push_back("auctioning_3");
    robot_state_strings.push_back("finished");
}

void Server::createLogFile() {
    int robot_id;
    std::string log_path;

    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("robot_id", robot_id))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("log_path", log_path))
        ROS_FATAL("INVALID PARAM");

    data_logger = new DataLogger("robot_state", (unsigned int)robot_id, log_path);
    std::string s = "robot_state.log";
    data_logger->createLogFile(s, "#sim_time,wall_time,robot_state");
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool Server::getRobotStateCallback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res) {
    mutex.lock();
    ROS_INFO_COND(LOG_SERVICE_CALL, "get_robot_state service required");
    res.robot_state = robot_state;
    ROS_INFO_COND(LOG_SERVICE_CALL, "Service response successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

bool Server::setRobotStateCallback(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res) {
    mutex.lock();
    ROS_INFO_COND(LOG_SERVICE_CALL, "set_robot_state service required");
//    if(state_locked) {
//        if(locking_node == req.setting_node)
//            transitionToNextStateIfPossible(req, res);
//        else {
//            ROS_ERROR("Node %s has required to set the robot state, but the robot state is locked by node %s! Setting failed", req.setting_node.c_str(), locking_node.c_str());
//            res.set_succeeded = false;
//        }
//    } else
        transitionToNextStateIfPossible(req, res);
    ROS_INFO_COND(LOG_SERVICE_CALL, "Service response successfully sent");
    mutex.unlock();
    return true;
}

void Server::transitionToNextStateIfPossible(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res) {
    if(isNewStateValid(req.robot_state)) {
        //TODO check if the transition is a valid one... but here or in testing???
        ROS_DEBUG("Robot state transiction: %s -> %s", robotStateEnumToString(robot_state).c_str(), robotStateEnumToString(req.robot_state).c_str());
        updateLogFile();
        robot_state = req.robot_state;
        res.set_succeeded = true;
    }
    else {
        ROS_ERROR("The next required robot state is invalid! Keeping current state (%s)", robotStateEnumToString(robot_state).c_str());
        res.set_succeeded = false;
    }
}

void Server::updateLogFile() {
    std::stringstream stream;
    stream << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << "," << robotStateEnumToString(robot_state).c_str() << std::endl;
    data_logger->updateLogFile("robot_state.log", stream);
}

bool Server::tryToLockRobotStateCallback(robot_state::TryToLockRobotState::Request &req, robot_state::TryToLockRobotState::Response &res) {
    mutex.lock();
    ROS_INFO_COND(LOG_SERVICE_CALL, "try_to_lock_robot_state service required");
    if(!state_locked) {
        if(req.locking_node.empty()) {
            ROS_ERROR("Lock on robot state required by an unnamed node: request rejected");
            res.lock_acquired = false;
        }
        else {
            counter++;
            if(counter > 1000000)
                counter = 0;
            state_locked = true;
            locking_node = req.locking_node;
            res.lock_acquired = true;        
            res.counter = counter;
            ROS_INFO_COND(LOG_LOCKING_INFO, "Lock on robot state acquired by %s", req.locking_node.c_str());
        }
    }
    else {
        ROS_WARN("The robot state is currently under control of %s: locking from %s failed", locking_node.c_str(), req.locking_node.c_str());
        res.lock_acquired = false;
    }
    ROS_INFO_COND(LOG_SERVICE_CALL, "Service response successfully sent");
    mutex.unlock();
    return true;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool Server::unlockRobotStateCallback(robot_state::UnlockRobotState::Request &req, robot_state::UnlockRobotState::Response &res) {
    mutex.lock();
    ROS_INFO_COND(LOG_SERVICE_CALL, "unlock_robot_state service required");
    state_locked = false;
    res.lock_released = true;
    res.counter = counter;
    ROS_INFO_COND(LOG_LOCKING_INFO, "Lock on robot state released");
    ROS_INFO_COND(LOG_SERVICE_CALL, "Service response successfully sent");
    mutex.unlock();
    return true;
}
#pragma GCC diagnostic pop

bool Server::isNewStateValid(int new_state) {
    return new_state >= 0 && (unsigned int)new_state < robot_state_strings.size();
}

std::string Server::robotStateEnumToString(unsigned int enum_value) {
    // Sanity check
    if(enum_value >= robot_state_strings.size())
        ROS_FATAL("Invalid robot state! It will cause a segmentation fault!");

    return robot_state_strings.at(enum_value);
}

