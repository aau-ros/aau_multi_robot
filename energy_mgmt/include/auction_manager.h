#ifndef AUCTION_MANAGER_H
#define AUCTION_MANAGER_H

#include <boost/thread.hpp>
#include <ros/ros.h>

class AuctionManager {
public:
    AuctionManager();
    void startNewAuction();
    bool isRobotParticipatingToAuction();

private:
    bool participating_to_auction;
    double auction_timeout;
    boost::mutex auction_mutex;
    ros::NodeHandle nh;
    ros::Timer terminate_auction_timer;

    void scheduleAuctionTermination();
    void terminateAuctionCallback(const ros::TimerEvent &event);
    void computeAuctionWinner();
};

#endif // AUCTION_MANAGER_H
