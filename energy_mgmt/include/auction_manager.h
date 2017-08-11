#ifndef AUCTION_MANAGER_H
#define AUCTION_MANAGER_H

#include <limits> // std::numeric_limits
#include <boost/thread.hpp> // boost::mutex
#include <ros/ros.h>
#include <adhoc_communication/SendEmAuction.h>
#include "bid_computer.h"
#include <utilities/time_manager_interface.h>

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
    double starting_time;
};

class AuctionManager {
public:
    AuctionManager(unsigned int robot_id);
    void setBidComputer(BidComputer *bid_computer);
    void setTimeManager(TimeManagerInterface *time_manager);
    void tryToAcquireDs();
    bool isRobotParticipatingToAuction();
    bool isRobotWinnerOfMostRecentAuction();
    void scheduleNextAuction();
    void preventParticipationToAuctions();
    void allowParticipationToAuctions();

    unsigned int _u1, _u2, _u3, _u4, _u5;
    bool _b1, _b2, _b3, _b4;
    int _d1;

private:
    BidComputer *bid_computer;
    TimeManagerInterface *time_manager;
    double auction_timeout, reauctioning_timeout, extra_auction_time;
    auction_participation_state_t auction_participation_state;
    bool winner_of_auction;
    boost::mutex auction_mutex;
    ros::NodeHandle nh;
    ros::Timer terminate_auction_timer, timer_restart_auction;
    unsigned int optimal_ds_id;
    unsigned int robot_id;
    std::vector<bid_t> auction_bids;
    ros::Subscriber auction_result_sub, auction_reply_sub, auction_starting_sub;
    ros::ServiceClient sc_send_auction;
    auction_t current_auction;
    bool robot_cannot_participate_to_auctions;
    bool optimal_ds_is_set;
    double sleep_time_between_two_participations;
    double time_last_participation;
    unsigned int local_auction_id;
    unsigned int num_robots;

    void initializeVariables(unsigned int robot_id);
    void createSubscribers();
    void createServiceClients();
    bid_t startNewAuction();
    unsigned int nextAuctionId();
    void scheduleAuctionTermination();
    void sendBid(bid_t bid, std::string topic);
    void terminateAuctionCallback(const ros::TimerEvent &event);
    unsigned int computeAuctionWinner();
    bool isThisRobotTheWinner(unsigned int winner_id);
    void sendAuctionResult(bid_t bid);
    void auctionReplyCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void auctionStartingCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void auctionResultCallback(const adhoc_communication::EmAuction::ConstPtr &msg);
    void endAuctionParticipationCallback(const ros::TimerEvent &event);
    void restartAuctionCallback(const ros::TimerEvent &event);
};

#endif // AUCTION_MANAGER_H
