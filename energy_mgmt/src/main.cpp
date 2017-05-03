#include <ros/ros.h>
#include <ros/console.h>
#include <battery.h>
#include <battery_simulate.h>
#include <docking.h>
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "energy_mgmt");

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
    
    battery_simulate bat;
    bat.compute(); //force computation and publishing...
    bat.publish();
    boost::thread thr_battery(boost::bind(&battery_simulate::run, &bat)); 
    
    // coordinate docking of robots for recharging
    docking doc;
    doc.wait_for_explorer();
    //boost::thread thr_spin(boost::bind(&docking::spin, &doc));
    
    //doc.wait_battery_info(); //only ok if threat thr_battery is active
    
    //doc.join_all_multicast_groups();
    doc.start_join_timer();
    
    

// Frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);
    while(ros::ok()){
        // get updates from subscriptions
        ros::spinOnce();
        
        //bat.spin();
        //bat.compute();
        //bat.publish();
        
        ros::spinOnce();
        
        //doc.join_all_multicast_groups();
        
        doc.update_robot_position();
        
        doc.update_robot_state();
        
        doc.discover_docking_stations();
        
        doc.check_reachable_ds();
        
        doc.compute_optimal_ds();
        
        doc.send_robot();
        
        //doc.update_llh();
        
        //doc.recompute_MST();
        
        //doc.send_fake_msg();
        

        // sleep for 1/rate seconds
        loop_rate.sleep();
    }
    
    //thr_spin.interrupt();
    //thr_spin.join();

    return 0;
}
