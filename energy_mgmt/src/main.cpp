#include <ros/ros.h>
#include <ros/console.h>
//#include <battery.h>
#include "battery_simulate.h"
#include "docking.h"
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <utilities/time_manager.h>
#include <robot_state/robot_state_management.h>

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "ds_mgmt");
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // handle battery management for different robot platforms
    string platform;
    
    docking doc;
    doc.create_log_files();
    doc.wait_for_explorer();  
    doc.wait_battery_info(); //only ok if threat thr_battery is active
    boost::thread thr_ds_management(boost::bind(&docking::ds_management, &doc));  

    // Frequency of loop
    double rate = 1; // Hz
    ros::Rate loop_rate(rate);
    ros::NodeHandle nh;
      
    while(ros::ok() && !doc.finished_bool){
        ros::spinOnce();

        doc.update_robot_position();
        doc.send_robot();
        doc.update_reamining_distance();
//        doc.recompute_MST();

        // Sleep for 1/rate seconds
        loop_rate.sleep();        
        ROS_INFO("End of main loop");
    }

    ROS_INFO("shutting down...");
    ros::shutdown();
    
    while(ros::ok()) //just to keep the node going but without doing nothing... used for collecting simulation data, can be removed otherwise
        ros::Duration(10).sleep();
        
    return 0;
}
