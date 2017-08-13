#ifndef CONCRETE_SENDER_H
#define CONCRETE_SENDER_H

#include "sender.h"

class ConcreteSender : public Sender {
public:
    ConcreteSender();
    void sendBid(bid_t bid, auction_t auction, std::string topic);
};

# endif // CONCRETE_SENDER_H
