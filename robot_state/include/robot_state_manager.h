#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/topic.h>
#include "robot_state/robot_state.h"
#include "robot_state/GetRobotState.h"

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

    ros::ServiceServer ss_get_robot_state;
    ros::ServiceServer ss_lock_robot_state;
    ros::ServiceServer ss_unlock_robot_state;

    void createServices();
    bool callback_get_robot_state(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res);
};

#endif /* ROBOT_STATE_MANAGER_H */
