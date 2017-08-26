#include <ros/ros.h>
#include <ros/console.h>
#include "docking.h"
#include <boost/thread.hpp>
#include "robot_state_manager.h"

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "ds_mgmt");
    ros::NodeHandle nh;
    ros::start();
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    RobotStateManager rsm("ds_mgmt");
    
    docking doc;
    doc.setRobotStateManager(&rsm);

    doc.create_log_files();
    
    ROS_INFO("calling wait_for_explorer");
    doc.wait_for_explorer();  
    
    ROS_INFO("wait_battery_info");
    doc.wait_battery_info(); //only ok if threat thr_battery is active
    
    ROS_INFO("thr_ds_management");
    boost::thread thr_ds_management(boost::bind(&docking::ds_management, &doc));  

    // Frequency of loop
    double rate = 1; // Hz
    ros::Rate loop_rate(rate);
      
    ROS_INFO("Entering main loop");
    while(ros::ok() && !doc.finished_bool){
        ros::spinOnce();

        doc.get_robot_state();
        doc.handle_robot_state();
        doc.update_robot_position();
        doc.send_robot();
        doc.update_reamining_distance();
//        doc.recompute_MST();

        // Sleep for 1/rate seconds
        loop_rate.sleep();        
        ROS_INFO("End of main loop");
    }
    
    thr_ds_management.join();
    thr_ds_management.interrupt();

    ROS_INFO("shutting down...");
    ros::Duration(3).sleep();
    ros::shutdown();
    
//    while(ros::ok()) //just to keep the node going but without doing nothing... used for collecting simulation data, can be removed otherwise
//        ros::Duration(10).sleep();
        
    return 0;
}
