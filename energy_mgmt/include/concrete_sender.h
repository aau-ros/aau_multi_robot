#ifndef CONCRETE_SENDER_H
#define CONCRETE_SENDER_H

#include "sender.h"

class ConcreteSender : public Sender {
public:
    ConcreteSender();
    void sendResults(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id, std::vector<unsigned int> participants);
    void sendBid(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id);
};

# endif // CONCRETE_SENDER_H
