#ifndef AUCTION_MANAGER_H
#define AUCTION_MANAGER_H

#include <limits> // std::numeric_limits
#include <boost/thread.hpp> // boost::mutex
#include <ros/ros.h>
#include <adhoc_communication/SendEmAuction.h>
#include "bid_computer.h"

enum auction_participation_state_t {
    IDLE,
    PARTICIPATING,
    MANAGING
};

//TODO put this here or inside the class? where is better?
struct auction_t
{
    unsigned int auction_id;
    double starting_time;
};

struct bid_t //TODO but in adhoc it is called auction, not bid...
{
    unsigned int auction_id;
    unsigned int robot_id;
    unsigned int ds_id;
    double bid;
};

class AuctionManager {
public:
    AuctionManager();
    void tryToAcquireDs();
    bool isRobotParticipatingToAuction();
    bool isRobotWinnerOfMostRecentAuction();

private:
    BidComputer bc;
    double auction_timeout, reauctioning_timeout, extra_auction_time;
    auction_participation_state_t auction_participation_state;
    bool winner_of_auction;
    boost::mutex auction_mutex;
    ros::NodeHandle nh;
    ros::Timer terminate_auction_timer, timer_restart_auction;
    unsigned int optimal_ds_id;
    unsigned int robot_id;
    std::vector<bid_t> auction_bids;
    bool auction_aborted;
    ros::Subscriber auction_result_sub, auction_reply_sub, auction_starting_sub;
    ros::ServiceClient sc_send_auction;
    auction_t current_auction;
    bool robot_cannot_participate_to_auctions;
    bool optimal_ds_is_set;

    void initializeVariables();
    void createSubscribers();
    void createServiceClients();
    bid_t startNewAuction();
    void scheduleAuctionTermination();
    void sendBid(bid_t bid, std::string topic);
    void terminateAuctionCallback(const ros::TimerEvent &event);
    unsigned int computeAuctionWinner();
    bool isThisRobotTheWinner(unsigned int winner_id);
    void sendAuctionResult(bid_t bid);
    void auctionReplyCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void auctionStartingCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void auctionResultCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void end_auction_participation_timer_callback(const ros::TimerEvent &event);

    void preventParticipationToAuctions();
    void allowParticipationToAuctions();
};

#endif // AUCTION_MANAGER_H
