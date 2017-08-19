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
  
    ros::init(argc, argv, "battery_mgmt");
    
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

    ros::NodeHandle nh_tilde("~");
    std::string robot_prefix;
    if(!nh_tilde.getParam("robot_prefix", robot_prefix))
        ROS_FATAL("robot_prefix not found!");

    RobotStateManager rsm(robot_prefix);

    battery_simulate bat;
    
    BatteryStateUpdater bsu(bat.getBatteryState()); //TODO not so beatiful...
    bsu.setRobotStateManager(&rsm);
    bsu.createLogDirectory();
    bsu.logMetadata();

    TimeManager tm;

    bat.setTimeManager(&tm);
    bat.setBatteryStateUpdater(&bsu);
    bat.createLogDirectory();
    bat.createLogFiles();

    double rate = 1; // Hz
    ros::Rate loop_rate(rate);
    ros::NodeHandle nh;

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
