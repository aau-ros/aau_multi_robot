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
  
    ros::init(argc, argv, "energy_mgmt");
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    // handle battery management for different robot platforms
    string platform;
    
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

    RobotStateApi r;
    
//    TimeManager tm;
//    battery_simulate bat;
//    bat.setTimeManager(&tm);
//    bat.initializeSimulationTime();
//    bat.createLogDirectory();
//    bat.createLogFiles();
//    boost::thread thr_battery(boost::bind(&battery_simulate::run, &bat)); 
//    
//    // coordinate docking of robots for recharging
    docking doc;
//    doc.create_log_files();
//    doc.wait_for_explorer();
//    
//    //doc.wait_battery_info(); //only ok if threat thr_battery is active
//    
//    //doc.join_all_multicast_groups();
//    //doc.start_join_timer();
//    
//    
//    boost::thread thr_spin(boost::bind(&docking::spin, &doc));
//    boost::thread thr_ds_management(boost::bind(&docking::ds_management, &doc));
//    

//// Frequency of loop
//    double rate = 1; // Hz
//    ros::Rate loop_rate(rate);
//    ros::NodeHandle nh;
//    //ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
//    
//    ros::AsyncSpinner spinner(20);
//    spinner.start();
//    
    while(ros::ok() && !doc.finished_bool){
//        // get updates from subscriptions
        ros::spinOnce();
//        
////        doc.test();
//        
////        bat.compute(); //force computation and publishing...
////        bat.publish();
////        bat.log();
//        
////        doc.join_all_multicast_groups();
//        
//        doc.update_robot_position();
//        
//        doc.update_robot_state();
//        
//        doc.send_robot();
//        
//        doc.update_reamining_distance();

//        
////        ros::spinOnce();
//        
////        doc.update_llh();
//        
////        doc.recompute_MST();
//        
////        doc.send_fake_msg();
//        

//        // Sleep for 1/rate seconds
//        loop_rate.sleep();
//        
//        ROS_INFO("End of main loop");
//        
//        /*
//        geometry_msgs::Twist cmd_vel;
//        cmd_vel.linear.x = 0.0;
//        cmd_vel.linear.y = 0.0;
//        //cmd_vel.linear.z = 0.0;
//        //cmd_vel.angular.x = 0.0;
//        //cmd_vel.angular.y = 0.0;
//        cmd_vel.angular.z = 0.5;
//        ROS_ERROR("%s",  pub_cmd_vel.getTopic().c_str());
//        pub_cmd_vel.publish(cmd_vel);
//        */
    }
//    
//    ROS_INFO("shutting down...");
//    
//    thr_battery.interrupt();
//    thr_battery.join();
//    thr_spin.interrupt();
//    thr_spin.join();
//    thr_ds_management.interrupt();
//    thr_ds_management.join();
//    
//    spinner.stop();
//    
//    ros::shutdown();
    
//    while(ros::ok()) //just to keep the node going but without doing nothing... used for collecting simulation data, can be removed otherwise
//        ros::Duration(10).sleep();
        
    return 0;
}
