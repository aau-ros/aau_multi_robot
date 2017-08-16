#ifndef AUCTION_MANAGER_INTERFACE_H
#define AUCTION_MANAGER_INTERFACE_H

#include <limits> // std::numeric_limits
#include <boost/thread.hpp> // boost::mutex
#include <ros/ros.h>
#include <utilities/time_manager_interface.h>
#include "bid_computer.h"
//#include "auction_structs.h"
#include "sender.h"

enum auction_participation_state_t {
    IDLE,
    PARTICIPATING,
    MANAGING
};

class AuctionManagerInterface {
public:
    void setBidComputer(BidComputer *bid_computer);
    void setTimeManager(TimeManagerInterface *time_manager);
    void setSender(Sender *sender);
    virtual void tryToAcquireDs() = 0;
    virtual bool isRobotParticipatingToAuction() = 0;
    virtual bool isRobotWinnerOfMostRecentAuction() = 0;
    virtual void preventParticipationToAuctions() = 0;
    virtual void allowParticipationToAuctions() = 0;
    virtual void setOptimalDs(unsigned int optimal_ds_id) = 0;
    virtual auction_t getCurrentAuction() = 0;
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

#endif // AUCTION_MANAGER_INTERFACE_H
