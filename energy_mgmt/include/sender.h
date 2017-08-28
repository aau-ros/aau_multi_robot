#ifndef SENDER_H
#define SENDER_H

#include <ros/ros.h>
#include <adhoc_communication/SendEmAuction.h>
#include "auction_structs.h"

class Sender {
public:
    virtual void sendBid(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) = 0;

protected:
    ros::ServiceClient send_auction_sc;
};

#endif // SENDER_H
