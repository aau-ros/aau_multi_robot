#ifndef MOCK_SENDER_H
#define MOCK_SENDER_H

#include "sender.h"

class MockSender : public Sender {
public:
    MockSender();
    void sendBid(bid_t bid, auction_t auction, std::string topic);
};

#endif // MOCK_SENDER_H
