#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <utilities/time_manager.h>
#include <robot_state/robot_state_management.h>

#include "battery_simulate.h"
#include "battery_state_updater.h"
#include "robot_state_manager.h"

int main(int argc, char** argv)
{
    std::string node = "battery_mgmt";
    ros::init(argc, argv, node);
    ros::NodeHandle nh;
    ros::start();
    ROS_INFO("Starting node %s", node.c_str());
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // handle battery management for different robot platforms
    std::string platform;
    
    /*
    ros::get_environment_variable(platform, "ROBOT_PLATFORM");
    if(platform.compare("turtlebot") == 0){
        battery_turtle bat;
    }
    else if(platform.compare("pioneer3dx") == 0 || platform.compare("pioneer3at") == 0){
        battery_pioneer bat;
    }
    else{
        battery_simulate bat;
    }
    */

    explorer::battery_state bs;

    RobotStateManager rsm(node);

    TimeManager tm;

    battery_simulate bat;
    bat.setBatteryState(&bs);
    
    BatteryStateUpdater bsu(&bs); //TODO not so beatiful...
    bsu.setRobotStateManager(&rsm);
    bsu.setTimeManager(&tm);
    bsu.createLogDirectory();
    bsu.logMetadata();



    bat.setTimeManager(&tm);
    bat.setBatteryStateUpdater(&bsu);
    bat.createLogDirectory();
    bat.createLogFiles();

    double rate = 1; // Hz
    ros::Rate loop_rate(rate);

    ROS_INFO("Starting main loop");
    while(ros::ok()){
        ros::spinOnce();
  
        bat.updateBatteryState();
        bat.logBatteryState();        
        bat.publishBatteryState();

        // Sleep for 1/rate seconds
        loop_rate.sleep();
        ROS_INFO("End of main loop");
    }
    
    ROS_INFO("shutting down...");   
    ros::shutdown();
        
    return 0;
}
