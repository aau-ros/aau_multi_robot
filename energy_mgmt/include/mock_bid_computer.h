#ifndef MOCK_BID_COMPUTER_H
#define MOCK_BID_COMPUTER_H

#include <iostream>
#include <vector>
#include "bid_computer.h"

class MockBidComputer : public BidComputer
{
public:
    MockBidComputer();
    double getBid();
    void addBid(double bid);

private:
    unsigned int bid_index;
    std::vector<double> bids;
};

#endif // MOCK_BID_COMPUTER_H
