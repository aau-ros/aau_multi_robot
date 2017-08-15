#ifndef AUCTION_OBSERVER_H
#define AUCTION_OBSERVER_H

#include <utilities/time_manager.h>
#include "robot_state_manager2.h"
#include "auction_manager_interface.h"

class AuctionObserver {
public:
    AuctionObserver();
    void setAuctionManager(AuctionManagerInterface *auction_manager);
    void setTimeManager(TimeManagerInterface *time_manager);
    void setRobotStateManager(RobotStateManager2 *robot_state_manager);
    void actAccordingToRobotStateAndAuctionResult();
    void sanityChecks();
    bool error_1, error_2, error_3; //TODO very bad... use dep. inj. also here (instead of directly using ROS_ERRRO) or protected var.s

private:
    AuctionManagerInterface *auction_manager;
    TimeManagerInterface *time_manager;
    RobotStateManager2 *robot_state_manager;

    bool auction_already_started;
    double reauctioning_timeout, auction_timeout, extra_auction_time;
    unsigned int robot_state;

    void loadParameters();
    unsigned int getRobotState();
    void setRobotState(unsigned int robot_state);
};

#endif // AUCTION_OBSERVER_H
