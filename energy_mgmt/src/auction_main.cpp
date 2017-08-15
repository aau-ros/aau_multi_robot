#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread.hpp>
#include <utilities/time_manager.h>
#include "auction_manager.h"
#include "auction_observer.h"
#include "concrete_sender.h"
#include "concrete_bid_computer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_mgmt");
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    TimeManager tm;
    ConcreteBidComputer cbc;
    
    AuctionManager auction_manager(123); //TODO(IMPORTANT)
    auction_manager.setTimeManager(&tm);
    auction_manager.setBidComputer(&cbc);

    AuctionObserver auction_observer;
    auction_observer.setAuctionManager(&auction_manager);
    auction_observer.setTimeManager(&tm);
    
    double rate = 10; // Hz
    ros::Rate loop_rate(rate);
    ros::NodeHandle nh;
    
//    ros::AsyncSpinner spinner(20);
//    spinner.start();
    
    while(ros::ok()){
        ros::spinOnce();
        auction_observer.actAccordingToRobotStateAndAuctionResult();
        auction_observer.sanityChecks();
        loop_rate.sleep();
        ROS_INFO("End of main loop");
    }
            
    ROS_INFO("shutting down...");
    
//    spinner.stop();
    
    ros::shutdown();
        
    return 0;
}
