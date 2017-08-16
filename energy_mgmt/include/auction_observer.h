#ifndef AUCTION_OBSERVER_H
#define AUCTION_OBSERVER_H

#include <utilities/time_manager.h>
#include "robot_state_manager_interface.h"
#include "auction_manager_interface.h"
#include <adhoc_communication/EmDockingStation.h>
#include <robot_state/robot_state_management.h>
#include "robot_state/GetRobotState.h"
#include "robot_state/SetRobotState.h"
#include "robot_state/TryToLockRobotState.h"
#include "robot_state/UnlockRobotState.h"

class AuctionObserver {
public:
    AuctionObserver();
    void setAuctionManager(AuctionManagerInterface *auction_manager);
    void setTimeManager(TimeManagerInterface *time_manager);
    void setRobotStateManager(RobotStateManagerInterface *robot_state_manager);

    void actAccordingToRobotStateAndAuctionResult();
    void sanityChecks();
    bool error_1, error_2, error_3; //TODO very bad... use dep. inj. also here (instead of directly using ROS_ERRRO) or protected var.s

private:
    AuctionManagerInterface *auction_manager;
    TimeManagerInterface *time_manager;
    RobotStateManagerInterface *robot_state_manager;
    ros::Subscriber sub_new_optimal_ds;

    bool auction_already_started;
    double reauctioning_timeout, auction_timeout, extra_auction_time;
    unsigned int robot_state;
    bool new_victory;
    unsigned int last_auction_id;

    void loadParameters();
    unsigned int getRobotState();
    void setRobotState(unsigned int robot_state);
    void analyzeAuctionResult();
    void newOptimalDsCallback(const adhoc_communication::EmDockingStation::ConstPtr &msg);
};

#endif // AUCTION_OBSERVER_H
