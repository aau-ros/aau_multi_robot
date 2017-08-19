#ifndef BATTERY_SIMULATE_H
#define BATTERY_SIMULATE_H

#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <fstream>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <explorer/battery_state.h>
#include <adhoc_communication/EmRobot.h>
#include <utilities/data_logger.h>
#include <utilities/time_manager.h>

#include "battery_state_updater_interface.h"

class battery_simulate
{
public:
    battery_simulate();
    void setTimeManager(TimeManagerInterface *time_manager);
    void setBatteryStateUpdater(BatteryStateUpdaterInterface *battery_state_updater);
    void updateBatteryState();
    void publishBatteryState();   
    void logBatteryState();
    void createLogDirectory();
    void createLogFiles();
    void setBatteryState(explorer::battery_state *state);

private:
    ros::NodeHandle nh;
    explorer::battery_state *state;

    ros::ServiceClient set_robot_state_sc;
    ros::ServiceClient get_robot_state_sc;

    ros::Time sim_time_start;
    ros::WallTime wall_time_start;

    ros::Publisher pub_battery;
    
    std::string log_path;
    std::string info_file, battery_state_filename;
    std::fstream fs_info, battery_state_fs;
    std::string robot_name;
    std::string robot_prefix;
    int robot_id;

    TimeManagerInterface *time_manager;
    BatteryStateUpdaterInterface *battery_state_updater;
    DataLogger *data_logger;
    
    void loadParameters();
    void initializeVariables();
    void initializeBatteryState();
    void initializeRobotName();
};


#endif //BATTERY_SIMULATE_H
