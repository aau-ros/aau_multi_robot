#ifndef SENDER_H
#define SENDER_H

#include <ros/ros.h>
#include <adhoc_communication/SendEmAuction.h>
#include <adhoc_communication/SendEmAuctionResult.h>
#include "auction_structs.h"

class Sender {
public:
    virtual void sendNewAuction(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) = 0;
    virtual void sendBid(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) = 0;
    virtual void sendResults(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id, std::vector<unsigned int> participants) = 0;

protected:
    ros::ServiceClient send_auction_sc;
};

#endif // SENDER_H
