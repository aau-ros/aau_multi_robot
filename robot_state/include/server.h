#ifndef SERVER_H
#define SERVER_H

#include <ros/ros.h>
#include <ros/package.h> //TODO
#include <ros/topic.h> //TODO
#include <boost/thread/mutex.hpp>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>
#include <robot_state/TryToLockRobotState.h>
#include <robot_state/UnlockRobotState.h>
#include <utilities/data_logger.h>
#include "robot_state.h"

//TODO ???
#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#define LOG_ALL     true
#define LOG_SERVICE_CALL false
#define LOG_LOCKING_INFO true

class Server
{
public:
    Server();

private:
    unsigned int robot_state;
    std::vector <std::string> robot_state_strings;
    boost::mutex mutex;
    bool state_locked;
    std::string locking_node;
    unsigned int counter = 0;
    DataLogger *data_logger;
    
    ros::ServiceServer get_robot_state_ss;
    ros::ServiceServer set_robot_state_ss;
    ros::ServiceServer try_to_lock_robot_state_ss;
    ros::ServiceServer unlock_robot_state_ss;

    void loadParameters();
    void initializeRobotState();
    void createServices();
    void fillRobotStateStringsVector();
    bool isNewStateValid(int new_state);
    void createLogFile();
    void updateLogFile();

    bool getRobotStateCallback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res);
    bool setRobotStateCallback(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res);
    void transitionToNextStateIfPossible(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res);
    bool tryToLockRobotStateCallback(robot_state::TryToLockRobotState::Request &req, robot_state::TryToLockRobotState::Response &res);
    bool unlockRobotStateCallback(robot_state::UnlockRobotState::Request &req, robot_state::UnlockRobotState::Response &res);

    std::string robotStateEnumToString(unsigned int enum_value);
    
};

#endif // SERVER_H
