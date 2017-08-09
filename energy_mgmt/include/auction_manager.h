#ifndef AUCTION_MANAGER_H
#define AUCTION_MANAGER_H

#include <limits> // std::numeric_limits
#include <boost/thread.hpp> // boost::mutex
#include <ros/ros.h>
#include "bid_computer.h"

//TODO put this here or inside the class? where is better?
struct auction_t
{
    int auction_id;
    double starting_time;
};

struct auction_bid_t
{
    int robot_id;
    float bid;
};

class AuctionManager {
public:
    AuctionManager();
    void startNewAuction();
    bool isRobotParticipatingToAuction();
    bool isRobotWinnerOfMostRecentAuction();

private:
    BidComputer bc;
    double auction_timeout, reauctioning_timeout, extra_auction_time;
    bool participating_to_auction;
    bool winner_of_auction;
    boost::mutex auction_mutex;
    ros::NodeHandle nh;
    ros::Timer terminate_auction_timer, timer_restart_auction;
    unsigned int optimal_ds_id;
    unsigned int robot_id;
    std::vector<auction_bid_t> auction_bids;
    bool auction_aborted;

    void createSubscribers();
    void scheduleAuctionTermination();
    void terminateAuctionCallback(const ros::TimerEvent &event);
    unsigned int computeAuctionWinner();
    bool isThisRobotTheWinner(unsigned int winner_id);
    void sendAuctionResult();
};

#endif // AUCTION_MANAGER_H
