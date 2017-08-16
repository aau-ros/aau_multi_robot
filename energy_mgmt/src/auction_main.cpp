#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread.hpp>
#include <utilities/time_manager.h>
#include "auction_manager.h"
#include "auction_observer.h"
#include "concrete_sender.h"
#include "concrete_bid_computer.h"
#include "robot_state_manager2.h"
#include "concrete_sender.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_mgmt");
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    TimeManager tm;
    ConcreteBidComputer cbc;
    RobotStateManager2 rsm;
    ConcreteSender cs;

    ros::NodeHandle nh_tilde("~");

    std::string robot_prefix;
    if(!nh_tilde.getParam("robot_prefix", robot_prefix))
        ROS_FATAL("robot_prefix not found!");

    unsigned int robot_id = atoi(robot_prefix.substr(7, 1).c_str());
    
    AuctionManager auction_manager(robot_id);
    auction_manager.setTimeManager(&tm);
    auction_manager.setBidComputer(&cbc);
    auction_manager.setSender(&cs);

    AuctionObserver auction_observer;
    auction_observer.setAuctionManager(&auction_manager);
    auction_observer.setRobotStateManager(&rsm);
    auction_observer.setTimeManager(&tm);
    
    double rate = 10; // Hz
    ros::Rate loop_rate(rate);
    ros::NodeHandle nh;
    
//    ros::AsyncSpinner spinner(20);
//    spinner.start();

    ros::Time last_udpate_llh = ros::Time::now();
    
    while(ros::ok()){
        ros::spinOnce();
        auction_observer.actAccordingToRobotStateAndAuctionResult();
        auction_observer.sanityChecks();

        if(ros::Time::now() - last_udpate_llh > ros::Duration(3)) {
            cbc.processMessages();
            cbc.updateLlh();
            last_udpate_llh = ros::Time::now();
        }

        loop_rate.sleep();
        ROS_INFO("End of main loop");
    }
            
    ROS_INFO("shutting down...");
    
//    spinner.stop();
    
    ros::shutdown();
        
    return 0;
}
