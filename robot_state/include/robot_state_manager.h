#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h> //TODO
#include <ros/topic.h> //TODO
#include <boost/thread/mutex.hpp>
#include "data_logger/data_logger.h"
#include "robot_state/robot_state.h"
#include "robot_state/GetRobotState.h"
#include "robot_state/SetRobotState.h"

//TODO ???
#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#define LOG_TRUE    true
#define LOG_FALSE   false
#define LOG_ALL     true

class RobotStateManager
{
public:
    RobotStateManager();

private:
    unsigned int robot_state;
    std::vector <std::string> robot_state_strings;
    boost::mutex mutex;
    
    ros::ServiceServer ss_get_robot_state; //TODO get_robot_state_ss
    ros::ServiceServer ss_set_robot_state;
    ros::ServiceServer ss_lock_robot_state;
    ros::ServiceServer ss_unlock_robot_state;

    void initializeRobotState();
    void createServices();
    void fillRobotStateStringsVector();    

    bool get_robot_state_callback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res); //TODO getRobotStateCallback
    bool set_robot_state_callback(robot_state::SetRobotState::Request &req, robot_state::SetRobotState::Response &res);

    std::string robotStateEnumToString(unsigned int enum_value);
    
};

#endif /* ROBOT_STATE_MANAGER_H */
